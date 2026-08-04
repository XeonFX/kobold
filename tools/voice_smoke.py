#!/usr/bin/env python3
"""Resample a mono WAV to 16 kHz and exercise the pinned Silero VAD model."""

from __future__ import annotations

import argparse
import wave
from pathlib import Path

import numpy as np
import onnxruntime as ort


def read_wav(path: Path) -> tuple[np.ndarray, int]:
    with wave.open(str(path), "rb") as source:
        if source.getnchannels() != 1 or source.getsampwidth() != 2:
            raise ValueError("expected mono 16-bit PCM WAV")
        rate = source.getframerate()
        samples = np.frombuffer(source.readframes(source.getnframes()), np.int16)
    return samples.astype(np.float32) / 32768.0, rate


def resample_linear(audio: np.ndarray, source_rate: int, target_rate: int) -> np.ndarray:
    if source_rate == target_rate:
        return audio
    output_size = round(audio.size * target_rate / source_rate)
    source_positions = np.arange(audio.size, dtype=np.float64)
    target_positions = np.arange(output_size, dtype=np.float64) * source_rate / target_rate
    return np.interp(target_positions, source_positions, audio).astype(np.float32)


def write_wav(path: Path, audio: np.ndarray, rate: int) -> None:
    pcm = np.clip(audio * 32768.0, -32768, 32767).astype(np.int16)
    with wave.open(str(path), "wb") as target:
        target.setnchannels(1)
        target.setsampwidth(2)
        target.setframerate(rate)
        target.writeframes(pcm.tobytes())


def vad_probabilities(session: ort.InferenceSession, audio: np.ndarray) -> list[float]:
    frame_size = 512
    state = np.zeros((2, 1, 128), dtype=np.float32)
    probabilities: list[float] = []
    for offset in range(0, audio.size, frame_size):
        frame = audio[offset : offset + frame_size]
        if frame.size < frame_size:
            frame = np.pad(frame, (0, frame_size - frame.size))
        probability, state = session.run(
            None,
            {
                "input": frame[np.newaxis, :],
                "state": state,
                "sr": np.array(16000, dtype=np.int64),
            },
        )
        probabilities.append(float(probability[0, 0]))
    return probabilities


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_wav", type=Path)
    parser.add_argument("vad_model", type=Path)
    parser.add_argument("output_wav", type=Path)
    args = parser.parse_args()

    audio, source_rate = read_wav(args.input_wav)
    audio_16k = resample_linear(audio, source_rate, 16000)
    write_wav(args.output_wav, audio_16k, 16000)

    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    session = ort.InferenceSession(
        str(args.vad_model),
        sess_options=options,
        providers=["CPUExecutionProvider"],
    )
    speech = vad_probabilities(session, audio_16k)
    silence = vad_probabilities(session, np.zeros(16000, dtype=np.float32))

    print(f"source_rate_hz={source_rate}")
    print(f"duration_s={audio_16k.size / 16000:.3f}")
    print(f"vad_frames={len(speech)}")
    print(f"vad_speech_max={max(speech):.6f}")
    print(f"vad_speech_mean={np.mean(speech):.6f}")
    print(f"vad_frames_over_0_5={sum(value >= 0.5 for value in speech)}")
    print(f"vad_silence_max={max(silence):.6f}")
    if max(speech) < 0.5 or max(silence) >= 0.5:
        raise RuntimeError("VAD did not separate synthesized speech from silence")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
