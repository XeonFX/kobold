#include <rknn_api.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

void check(int rc, const char* operation) {
  if (rc != RKNN_SUCC) {
    throw std::runtime_error(std::string(operation) + " failed with " +
                             std::to_string(rc));
  }
}

std::vector<std::uint8_t> read_file(const std::string& path) {
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream) {
    throw std::runtime_error("cannot open model: " + path);
  }
  const auto length = stream.tellg();
  if (length <= 0) {
    throw std::runtime_error("model is empty: " + path);
  }
  std::vector<std::uint8_t> data(static_cast<std::size_t>(length));
  stream.seekg(0);
  stream.read(reinterpret_cast<char*>(data.data()), length);
  if (!stream) {
    throw std::runtime_error("short read from model: " + path);
  }
  return data;
}

double percentile(std::vector<double> values, double fraction) {
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(
      std::llround(fraction * static_cast<double>(values.size() - 1)));
  return values[index];
}

struct ContextGuard {
  rknn_context value{};
  ~ContextGuard() {
    if (value != 0) {
      rknn_destroy(value);
    }
  }
};

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 2 || argc > 5) {
      std::cerr << "usage: " << argv[0]
                << " MODEL [ITERATIONS=300] [WARMUP=20] [raw|float]\n";
      return 2;
    }

    const std::string model_path = argv[1];
    const int iterations = argc > 2 ? std::stoi(argv[2]) : 300;
    const int warmup = argc > 3 ? std::stoi(argv[3]) : 20;
    const std::string output_mode = argc > 4 ? argv[4] : "raw";
    if (iterations < 1 || warmup < 0 ||
        (output_mode != "raw" && output_mode != "float")) {
      throw std::runtime_error("invalid benchmark arguments");
    }
    const bool want_float = output_mode == "float";

    const auto init_start = Clock::now();
    auto model = read_file(model_path);
    ContextGuard context;
    check(rknn_init(&context.value, model.data(),
                    static_cast<std::uint32_t>(model.size()), 0, nullptr),
          "rknn_init");
    check(rknn_set_core_mask(context.value, RKNN_NPU_CORE_0),
          "rknn_set_core_mask");
    const auto init_end = Clock::now();

    rknn_sdk_version version{};
    check(rknn_query(context.value, RKNN_QUERY_SDK_VERSION, &version,
                     sizeof(version)),
          "rknn_query(SDK_VERSION)");

    rknn_input_output_num io{};
    check(rknn_query(context.value, RKNN_QUERY_IN_OUT_NUM, &io, sizeof(io)),
          "rknn_query(IN_OUT_NUM)");
    if (io.n_input != 1 || io.n_output == 0) {
      throw std::runtime_error("benchmark expects one input and at least one output");
    }

    rknn_tensor_attr input_attr{};
    input_attr.index = 0;
    check(rknn_query(context.value, RKNN_QUERY_INPUT_ATTR, &input_attr,
                     sizeof(input_attr)),
          "rknn_query(INPUT_ATTR)");

    std::vector<rknn_tensor_attr> output_attrs(io.n_output);
    for (std::uint32_t i = 0; i < io.n_output; ++i) {
      output_attrs[i].index = i;
      check(rknn_query(context.value, RKNN_QUERY_OUTPUT_ATTR, &output_attrs[i],
                       sizeof(output_attrs[i])),
            "rknn_query(OUTPUT_ATTR)");
    }

    // This benchmark deliberately feeds the same 8-bit NHWC tensor as the
    // RKNNLite benchmark. Preprocessing is outside both timed regions.
    std::vector<std::uint8_t> input_data(input_attr.n_elems, 0);
    rknn_input input{};
    input.index = 0;
    input.buf = input_data.data();
    input.size = static_cast<std::uint32_t>(input_data.size());
    input.pass_through = 0;
    input.type = RKNN_TENSOR_UINT8;
    input.fmt = RKNN_TENSOR_NHWC;

    volatile double checksum = 0.0;
    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>(iterations));
    std::vector<std::uint32_t> output_sizes(io.n_output);

    for (int iteration = -warmup; iteration < iterations; ++iteration) {
      std::vector<rknn_output> outputs(io.n_output);
      for (std::uint32_t i = 0; i < io.n_output; ++i) {
        outputs[i].index = i;
        outputs[i].want_float = want_float ? 1 : 0;
        outputs[i].is_prealloc = 0;
      }

      const auto start = Clock::now();
      check(rknn_inputs_set(context.value, 1, &input), "rknn_inputs_set");
      check(rknn_run(context.value, nullptr), "rknn_run");
      check(rknn_outputs_get(context.value, io.n_output, outputs.data(), nullptr),
            "rknn_outputs_get");
      const auto end = Clock::now();

      for (std::uint32_t i = 0; i < io.n_output; ++i) {
        output_sizes[i] = outputs[i].size;
        if (outputs[i].buf != nullptr && outputs[i].size > 0) {
          checksum += want_float
                          ? static_cast<double>(
                                static_cast<float*>(outputs[i].buf)[0])
                          : static_cast<double>(
                                static_cast<std::int8_t*>(outputs[i].buf)[0]);
        }
      }
      check(rknn_outputs_release(context.value, io.n_output, outputs.data()),
            "rknn_outputs_release");

      if (iteration >= 0) {
        samples.push_back(
            std::chrono::duration<double, std::milli>(end - start).count());
      }
    }

    const double mean =
        std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
    double squared_error = 0.0;
    for (const double sample : samples) {
      squared_error += (sample - mean) * (sample - mean);
    }
    const double stddev = std::sqrt(squared_error / samples.size());
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);

    std::cout << std::fixed << std::setprecision(3)
              << "implementation=c++-rknn-c-api\n"
              << "api_version=" << version.api_version << "\n"
              << "driver_version=" << version.drv_version << "\n"
              << "core_mask=1\n"
              << "output_mode=" << output_mode << "\n"
              << "iterations=" << iterations << "\n"
              << "warmup=" << warmup << "\n"
              << "init_ms="
              << std::chrono::duration<double, std::milli>(init_end - init_start)
                     .count()
              << "\n"
              << "mean_ms=" << mean << "\n"
              << "stddev_ms=" << stddev << "\n"
              << "min_ms=" << *std::min_element(samples.begin(), samples.end())
              << "\n"
              << "p50_ms=" << percentile(samples, 0.50) << "\n"
              << "p95_ms=" << percentile(samples, 0.95) << "\n"
              << "p99_ms=" << percentile(samples, 0.99) << "\n"
              << "max_ms=" << *std::max_element(samples.begin(), samples.end())
              << "\n"
              << "fps=" << 1000.0 / mean << "\n"
              << "max_rss_kib=" << usage.ru_maxrss << "\n"
              << "output_bytes=";
    for (std::size_t i = 0; i < output_sizes.size(); ++i) {
      std::cout << (i == 0 ? "" : ",") << output_sizes[i];
    }
    std::cout << "\nchecksum=" << checksum << "\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << "\n";
    return 1;
  }
}
