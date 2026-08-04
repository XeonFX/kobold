#!/usr/bin/env python3
"""Benchmark RKNNLite with the same input/core/timing policy as the C++ test."""

from __future__ import annotations

import argparse
import resource
import statistics
import time

import numpy as np
from rknnlite.api import RKNNLite


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[round(fraction * (len(ordered) - 1))]


def check(result: int, operation: str) -> None:
    if result != 0:
        raise RuntimeError(f"{operation} failed with {result}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("model")
    parser.add_argument("iterations", type=int, nargs="?", default=300)
    parser.add_argument("warmup", type=int, nargs="?", default=20)
    args = parser.parse_args()
    if args.iterations < 1 or args.warmup < 0:
        parser.error("iterations must be positive and warmup non-negative")

    init_start = time.perf_counter_ns()
    runtime = RKNNLite(verbose=False)
    check(runtime.load_rknn(args.model), "load_rknn")
    check(runtime.init_runtime(core_mask=RKNNLite.NPU_CORE_0), "init_runtime")
    init_ms = (time.perf_counter_ns() - init_start) / 1_000_000

    # RKNNLite requires the explicit batch dimension for this model. It returns
    # dequantized float32 arrays, so compare it with C++ `float` output mode.
    input_data = np.zeros((1, 640, 640, 3), dtype=np.uint8)
    samples: list[float] = []
    checksum = 0.0
    output_bytes: list[int] = []

    try:
        for iteration in range(-args.warmup, args.iterations):
            start = time.perf_counter_ns()
            outputs = runtime.inference(
                inputs=[input_data], data_format=["nhwc"]
            )
            elapsed_ms = (time.perf_counter_ns() - start) / 1_000_000
            if outputs is None:
                raise RuntimeError("inference returned no outputs")
            output_bytes = [int(output.nbytes) for output in outputs]
            checksum += sum(float(output.flat[0]) for output in outputs)
            if iteration >= 0:
                samples.append(elapsed_ms)
    finally:
        runtime.release()

    print("implementation=python-rknnlite")
    print("core_mask=1")
    print("output_mode=float")
    print(f"iterations={args.iterations}")
    print(f"warmup={args.warmup}")
    print(f"init_ms={init_ms:.3f}")
    print(f"mean_ms={statistics.fmean(samples):.3f}")
    print(f"stddev_ms={statistics.pstdev(samples):.3f}")
    print(f"min_ms={min(samples):.3f}")
    print(f"p50_ms={percentile(samples, 0.50):.3f}")
    print(f"p95_ms={percentile(samples, 0.95):.3f}")
    print(f"p99_ms={percentile(samples, 0.99):.3f}")
    print(f"max_ms={max(samples):.3f}")
    print(f"fps={1000 / statistics.fmean(samples):.3f}")
    print(f"max_rss_kib={resource.getrusage(resource.RUSAGE_SELF).ru_maxrss}")
    print("output_bytes=" + ",".join(str(size) for size in output_bytes))
    print(f"checksum={checksum:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
