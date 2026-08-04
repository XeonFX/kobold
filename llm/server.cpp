// Kobold RKLLM server — native OpenAI-compatible HTTP API for RK3588.
//
// The process owns the RKLLM handle directly.  There is no Python interpreter
// or cross-process token relay in the inference path.  RKLLM permits one run at
// a time for this model, so concurrent completion requests fail fast with 503.

#include <httplib.h>
#include <nlohmann/json.hpp>
#include <rkllm.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <condition_variable>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

using json = nlohmann::json;

namespace {

std::string env_or(const char* name, const char* fallback) {
  const char* value = std::getenv(name);
  return value && *value ? value : fallback;
}

int env_int(const char* name, int fallback) {
  const std::string value = env_or(name, "");
  if (value.empty()) return fallback;
  try {
    return std::stoi(value);
  } catch (...) {
    throw std::runtime_error(std::string("invalid integer in ") + name);
  }
}

std::string completion_id() {
  static std::atomic<unsigned long long> counter{0};
  std::ostringstream out;
  out << "chatcmpl-rkllm-" << std::time(nullptr) << '-'
      << counter.fetch_add(1, std::memory_order_relaxed);
  return out.str();
}

std::string tool_call_id(const std::string& completion) {
  const size_t suffix = completion.find("rkllm-");
  return "call-" + completion.substr(suffix == std::string::npos ? 0 : suffix);
}

int approximate_tokens(const std::string& text) {
  if (text.empty()) return 0;
  int words = 0;
  bool inside = false;
  for (const unsigned char ch : text) {
    const bool whitespace = ch == ' ' || ch == '\n' || ch == '\r' || ch == '\t';
    if (!whitespace && !inside) ++words;
    inside = !whitespace;
  }
  return std::max(words, 1);
}

json error_body(const std::string& message, const std::string& type = "invalid_request_error") {
  return {{"error", {{"message", message}, {"type", type}, {"param", nullptr}, {"code", nullptr}}}};
}

void json_response(httplib::Response& response, int status, const json& body) {
  response.status = status;
  response.set_content(body.dump(), "application/json");
}

bool authorized(const httplib::Request& request, const std::string& api_key) {
  if (api_key.empty()) return true;
  return request.get_header_value("Authorization") == "Bearer " + api_key;
}

struct RequestState {
  std::mutex mutex;
  std::condition_variable changed;
  std::deque<std::string> chunks;
  std::string output;
  RKLLMPerfStat perf{};
  bool finished = false;
  bool failed = false;
};

class RkllmEngine {
 public:
  RkllmEngine(std::string model_path, int max_context, int max_new_tokens)
      : model_path_(std::move(model_path)),
        max_context_(max_context),
        default_max_new_tokens_(max_new_tokens) {
    RKLLMParam parameters = rkllm_createDefaultParam();
    parameters.model_path = model_path_.c_str();
    parameters.max_context_len = max_context_;
    parameters.max_new_tokens = default_max_new_tokens_;
    parameters.n_keep = -1;
    parameters.top_k = 1;
    parameters.top_p = 0.9F;
    parameters.temperature = 0.8F;
    parameters.repeat_penalty = 1.1F;
    parameters.frequency_penalty = 0.0F;
    parameters.presence_penalty = 0.0F;
    parameters.skip_special_token = true;
    parameters.ignore_eos_token = false;
    parameters.is_async = false;
    parameters.extend_param.base_domain_id = 0;
    parameters.extend_param.embed_flash = 1;
    parameters.extend_param.n_batch = 1;
    parameters.extend_param.use_cross_attn = 0;
    parameters.extend_param.enabled_cpus_num = 4;
    parameters.extend_param.enabled_cpus_mask = CPU4 | CPU5 | CPU6 | CPU7;

    RKLLMCallback callbacks{};
    callbacks.result_callback = &RkllmEngine::result_callback;

    std::cerr << "kobold-llm: loading " << model_path_ << " context=" << max_context_
              << " max_new_tokens=" << default_max_new_tokens_ << std::endl;
    const int result = rkllm_init(&handle_, &parameters, &callbacks);
    if (result != 0 || handle_ == nullptr) {
      throw std::runtime_error("rkllm_init failed with code " + std::to_string(result));
    }

    // HTTP requests carry the complete conversation, so make inference
    // stateless and apply Qwen's chat template explicitly.  This avoids sharing
    // KV history between browsers/agents and makes retries deterministic.
    const int template_result = rkllm_set_chat_template(handle_, "", "", "");
    if (template_result != 0) {
      throw std::runtime_error("rkllm_set_chat_template failed with code " +
                               std::to_string(template_result));
    }
  }

  ~RkllmEngine() {
    if (handle_ != nullptr) rkllm_destroy(handle_);
  }

  RkllmEngine(const RkllmEngine&) = delete;
  RkllmEngine& operator=(const RkllmEngine&) = delete;

  std::mutex& run_mutex() { return run_mutex_; }
  int default_max_new_tokens() const { return default_max_new_tokens_; }

  int run(const std::string& prompt, bool enable_thinking, int max_new_tokens,
          RKLLMSamplingParam& sampling, RequestState& state) {
    const int clear_result = rkllm_clear_kv_cache(handle_, 0, nullptr, nullptr);
    if (clear_result != 0) {
      std::cerr << "kobold-llm: warning: rkllm_clear_kv_cache returned " << clear_result
                << std::endl;
    }

    RKLLMInput input{};
    input.role = "user";
    input.enable_thinking = enable_thinking;
    input.input_type = RKLLM_INPUT_PROMPT;
    input.prompt_input = prompt.c_str();

    RKLLMInferParam infer{};
    infer.mode = RKLLM_INFER_GENERATE;
    infer.sampling_params = &sampling;
    infer.keep_history = 0;
    infer.max_new_tokens = max_new_tokens;

    const int result = rkllm_run(handle_, &input, &infer, &state);
    if (result != 0) {
      std::lock_guard<std::mutex> lock(state.mutex);
      state.failed = true;
      state.finished = true;
      state.changed.notify_all();
    }
    return result;
  }

  void abort() { rkllm_abort(handle_); }

 private:
  static int result_callback(RKLLMResult* result, void* userdata, LLMCallState call_state) {
    auto* state = static_cast<RequestState*>(userdata);
    if (state == nullptr) return 0;

    std::lock_guard<std::mutex> lock(state->mutex);
    if (result != nullptr) state->perf = result->perf;

    if (call_state == RKLLM_RUN_NORMAL && result != nullptr && result->text != nullptr) {
      std::string text(result->text);
      state->output += text;
      state->chunks.push_back(std::move(text));
    } else if (call_state == RKLLM_RUN_FINISH) {
      state->finished = true;
    } else if (call_state == RKLLM_RUN_ERROR) {
      state->failed = true;
      state->finished = true;
    }
    state->changed.notify_all();
    return 0;
  }

  std::string model_path_;
  int max_context_;
  int default_max_new_tokens_;
  LLMHandle handle_ = nullptr;
  std::mutex run_mutex_;
};

std::string message_text(const json& message, bool& has_unsupported_content) {
  if (!message.contains("content") || message["content"].is_null()) return "";
  const json& content = message["content"];
  if (content.is_string()) return content.get<std::string>();
  if (!content.is_array()) {
    throw std::runtime_error("message content must be a string or an array");
  }

  std::string text;
  for (const auto& part : content) {
    if (!part.is_object()) continue;
    const std::string type = part.value("type", "");
    if (type == "text") {
      if (!text.empty()) text += '\n';
      text += part.value("text", "");
    } else {
      has_unsupported_content = true;
    }
  }
  return text;
}

std::string format_conversation(const json& messages, const json* tools, bool enable_thinking,
                                bool& has_unsupported_content) {
  if (!messages.is_array() || messages.empty()) {
    throw std::runtime_error("messages must be a non-empty array");
  }

  std::string system_prompt = "You are a helpful assistant.";
  for (const auto& message : messages) {
    if (message.value("role", "") == "system") {
      system_prompt = message_text(message, has_unsupported_content);
      break;
    }
  }

  if (tools != nullptr && tools->is_array() && !tools->empty()) {
    std::string tool_prompt =
        "# Tools\n\nYou have access to the following functions:\n\n<tools>\n";
    for (const auto& tool : *tools) tool_prompt += tool.dump() + "\n";
    tool_prompt +=
        "</tools>\n\nIf you call a function, use only this format with no suffix:\n\n"
        "<tool_call>\n<function=function_name>\n"
        "<parameter=parameter_name>\nvalue\n</parameter>\n"
        "</function>\n</tool_call>\n\n"
        "Required parameters must be present. If no function is needed, answer normally.";
    if (!system_prompt.empty()) tool_prompt += "\n\n" + system_prompt;
    system_prompt = std::move(tool_prompt);
  }

  std::string prompt = "<|im_start|>system\n" + system_prompt + "<|im_end|>\n";
  for (const auto& message : messages) {
    const std::string role = message.value("role", "");
    if (role == "system") continue;
    if (role != "user" && role != "assistant" && role != "tool") {
      throw std::runtime_error("unsupported message role: " + role);
    }
    if (role == "tool") {
      prompt += "<|im_start|>user\n<tool_response>\n";
      prompt += message_text(message, has_unsupported_content);
      prompt += "\n</tool_response><|im_end|>\n";
      continue;
    }
    prompt += "<|im_start|>" + role + "\n";
    std::string content = message_text(message, has_unsupported_content);
    if (role == "assistant" && message.contains("tool_calls") &&
        message["tool_calls"].is_array()) {
      for (const auto& tool_call : message["tool_calls"]) {
        if (!tool_call.is_object() || !tool_call.contains("function")) continue;
        const json& function = tool_call["function"];
        if (!function.is_object() || !function.contains("name")) continue;
        json arguments = json::object();
        if (function.contains("arguments")) {
          if (function["arguments"].is_string()) {
            arguments = json::parse(function["arguments"].get<std::string>(), nullptr, false);
            if (arguments.is_discarded()) arguments = function["arguments"];
          } else {
            arguments = function["arguments"];
          }
        }
        if (!content.empty()) content += "\n\n";
        content += "<tool_call>\n<function=" + function["name"].get<std::string>() + ">\n";
        if (arguments.is_object()) {
          for (auto argument = arguments.begin(); argument != arguments.end(); ++argument) {
            content += "<parameter=" + argument.key() + ">\n";
            content += argument.value().is_string() ? argument.value().get<std::string>()
                                                    : argument.value().dump();
            content += "\n</parameter>\n";
          }
        }
        content += "</function>\n</tool_call>";
      }
    }
    prompt += content;
    prompt += "<|im_end|>\n";
  }
  prompt += "<|im_start|>assistant\n";
  // Supplying a complete stateless template disables RKLLM's automatic
  // enable_thinking handling. Pre-close the reasoning block for normal robot
  // tool/chat turns so the token budget is spent on the answer.
  if (!enable_thinking) prompt += "<think>\n\n</think>\n\n";
  return prompt;
}

struct ParsedToolCall {
  std::string name;
  std::string arguments;
};

std::optional<ParsedToolCall> parse_tool_call(const std::string& output) {
  constexpr const char* marker = "<tool_call>";
  const size_t marker_position = output.find(marker);
  if (marker_position == std::string::npos) return std::nullopt;

  // Qwen3.5's official template uses XML-like function and parameter tags.
  constexpr const char* function_marker = "<function=";
  const size_t function_start = output.find(function_marker, marker_position);
  if (function_start != std::string::npos) {
    const size_t name_start = function_start + std::char_traits<char>::length(function_marker);
    const size_t name_end = output.find('>', name_start);
    if (name_end == std::string::npos || name_end == name_start) return std::nullopt;

    json arguments = json::object();
    constexpr const char* parameter_marker = "<parameter=";
    constexpr const char* parameter_close = "</parameter>";
    size_t cursor = name_end + 1;
    while (true) {
      const size_t parameter_start = output.find(parameter_marker, cursor);
      const size_t function_end = output.find("</function>", cursor);
      if (parameter_start == std::string::npos ||
          (function_end != std::string::npos && parameter_start > function_end)) {
        break;
      }
      const size_t key_start =
          parameter_start + std::char_traits<char>::length(parameter_marker);
      const size_t key_end = output.find('>', key_start);
      if (key_end == std::string::npos || key_end == key_start) return std::nullopt;
      const size_t value_start = key_end + 1;
      const size_t value_end = output.find(parameter_close, value_start);
      if (value_end == std::string::npos) return std::nullopt;

      std::string value = output.substr(value_start, value_end - value_start);
      const size_t first = value.find_first_not_of(" \t\r\n");
      const size_t last = value.find_last_not_of(" \t\r\n");
      value = first == std::string::npos ? "" : value.substr(first, last - first + 1);
      json typed_value = json::parse(value, nullptr, false);
      arguments[output.substr(key_start, key_end - key_start)] =
          typed_value.is_discarded() ? json(value) : typed_value;
      cursor = value_end + std::char_traits<char>::length(parameter_close);
    }
    return ParsedToolCall{output.substr(name_start, name_end - name_start), arguments.dump()};
  }

  // Retain compatibility with older/custom prompts that emit a JSON object.
  const size_t object_start =
      output.find('{', marker_position + std::char_traits<char>::length(marker));
  if (object_start == std::string::npos) return std::nullopt;

  bool quoted = false;
  bool escaped = false;
  int depth = 0;
  size_t object_end = std::string::npos;
  for (size_t index = object_start; index < output.size(); ++index) {
    const char character = output[index];
    if (quoted) {
      if (escaped) {
        escaped = false;
      } else if (character == '\\') {
        escaped = true;
      } else if (character == '"') {
        quoted = false;
      }
      continue;
    }
    if (character == '"') {
      quoted = true;
    } else if (character == '{') {
      ++depth;
    } else if (character == '}' && --depth == 0) {
      object_end = index;
      break;
    }
  }
  if (object_end == std::string::npos) return std::nullopt;

  try {
    const json call = json::parse(output.substr(object_start, object_end - object_start + 1));
    if (!call.is_object() || !call.contains("name") || !call["name"].is_string()) {
      return std::nullopt;
    }
    const json arguments = call.value("arguments", json::object());
    return ParsedToolCall{call["name"].get<std::string>(),
                          arguments.is_string() ? arguments.get<std::string>()
                                                : arguments.dump()};
  } catch (...) {
    return std::nullopt;
  }
}

json openai_tool_call(const ParsedToolCall& call, const std::string& id) {
  return {{"id", tool_call_id(id)},
          {"type", "function"},
          {"function", {{"name", call.name}, {"arguments", call.arguments}}}};
}

RKLLMSamplingParam sampling_from(const json& request) {
  RKLLMSamplingParam sampling{};
  sampling.top_k = std::clamp(request.value("top_k", 1), 1, 100);
  sampling.top_p = std::clamp(request.value("top_p", 0.9F), 0.0F, 1.0F);
  sampling.temperature = std::clamp(request.value("temperature", 0.8F), 0.0F, 2.0F);
  sampling.repeat_penalty = std::clamp(request.value("repeat_penalty", 1.1F), 0.0F, 2.0F);
  sampling.frequency_penalty =
      std::clamp(request.value("frequency_penalty", 0.0F), -2.0F, 2.0F);
  sampling.presence_penalty =
      std::clamp(request.value("presence_penalty", 0.0F), -2.0F, 2.0F);
  sampling.mirostat = 0;
  sampling.mirostat_tau = 5.0F;
  sampling.mirostat_eta = 0.1F;
  return sampling;
}

json stream_chunk(const std::string& id, const std::string& model,
                  const json& delta, const json& finish_reason) {
  return {{"id", id},
          {"object", "chat.completion.chunk"},
          {"created", std::time(nullptr)},
          {"model", model},
          {"choices", {{{"index", 0},
                         {"delta", delta},
                         {"logprobs", nullptr},
                         {"finish_reason", finish_reason}}}}};
}

struct StreamContext {
  std::shared_ptr<RequestState> state;
  std::shared_ptr<std::unique_lock<std::mutex>> engine_lock;
  std::thread worker;
  RkllmEngine* engine = nullptr;
  std::string id;
  std::string model;
  bool sent_role = false;
  bool sent_finish = false;
  bool output_mode_decided = false;
  bool tool_mode = false;
  std::string pending;
};

httplib::Server* signal_server = nullptr;

void stop_server(int) {
  if (signal_server != nullptr) signal_server->stop();
}

}  // namespace

int main() {
  try {
    const std::string model_path = env_or("MODEL_PATH", "/models/qwen3.5-2b_w8a8_rk3588.rkllm");
    const std::string model_name = env_or("MODEL_NAME", "qwen3.5-2b-rk3588");
    const std::string api_key = env_or("API_KEY", "");
    const int port = env_int("PORT", 8080);
    const int max_context = std::clamp(env_int("MAX_CONTEXT", 4096), 256, 16384);
    const int default_max_new_tokens =
        std::clamp(env_int("MAX_NEW_TOKENS", 512), 1, max_context);

    RkllmEngine engine(model_path, max_context, default_max_new_tokens);
    httplib::Server server;
    signal_server = &server;
    std::signal(SIGINT, stop_server);
    std::signal(SIGTERM, stop_server);

    auto require_auth = [&](const httplib::Request& request, httplib::Response& response) {
      if (authorized(request, api_key)) return true;
      json_response(response, 401, error_body("invalid API key", "authentication_error"));
      return false;
    };

    server.Get("/healthz", [&](const httplib::Request&, httplib::Response& response) {
      json_response(response, 200,
                    {{"ok", true},
                     {"model", model_name},
                     {"rkllm_runtime", "1.3.0"},
                     {"max_context", max_context}});
    });

    server.Get("/v1/models", [&](const httplib::Request& request, httplib::Response& response) {
      if (!require_auth(request, response)) return;
      json_response(response, 200,
                    {{"object", "list"},
                     {"data", {{{"id", model_name},
                                {"object", "model"},
                                {"created", std::time(nullptr)},
                                {"owned_by", "kobold-rkllm"}}}}});
    });

    server.Post("/v1/chat/completions",
                [&](const httplib::Request& http_request, httplib::Response& response) {
      if (!require_auth(http_request, response)) return;

      json request;
      try {
        request = json::parse(http_request.body);
      } catch (const std::exception& error) {
        json_response(response, 400, error_body(std::string("invalid JSON: ") + error.what()));
        return;
      }

      if (!request.contains("messages")) {
        json_response(response, 400, error_body("missing messages"));
        return;
      }

      bool unsupported_content = false;
      const bool enable_thinking = request.value("enable_thinking", false);
      std::string prompt;
      try {
        const json* tools = request.contains("tools") ? &request["tools"] : nullptr;
        prompt = format_conversation(request["messages"], tools, enable_thinking,
                                     unsupported_content);
      } catch (const std::exception& error) {
        json_response(response, 400, error_body(error.what()));
        return;
      }
      if (unsupported_content) {
        json_response(response, 400,
                      error_body("image/audio message parts are not yet supported by this endpoint"));
        return;
      }

      const bool stream = request.value("stream", false);
      const int max_tokens = std::clamp(
          request.value("max_tokens", engine.default_max_new_tokens()), 1, max_context);
      RKLLMSamplingParam sampling = sampling_from(request);
      const std::string response_model = request.value("model", model_name);
      const std::string id = completion_id();

      auto engine_lock =
          std::make_shared<std::unique_lock<std::mutex>>(engine.run_mutex(), std::try_to_lock);
      if (!engine_lock->owns_lock()) {
        json_response(response, 503,
                      error_body("RKLLM is busy; retry after the current completion", "server_error"));
        response.set_header("Retry-After", "1");
        return;
      }

      if (!stream) {
        RequestState state;
        const int run_result =
            engine.run(prompt, enable_thinking, max_tokens, sampling, state);
        if (run_result != 0 || state.failed) {
          json_response(response, 500,
                        error_body("RKLLM inference failed with code " +
                                       std::to_string(run_result),
                                   "server_error"));
          return;
        }
        const int prompt_tokens = state.perf.prefill_tokens > 0
                                      ? state.perf.prefill_tokens
                                      : approximate_tokens(prompt);
        const int completion_tokens = state.perf.generate_tokens > 0
                                          ? state.perf.generate_tokens
                                          : approximate_tokens(state.output);
        json message = {{"role", "assistant"}, {"content", state.output}};
        std::string finish_reason = "stop";
        if (const auto tool_call = parse_tool_call(state.output)) {
          message["content"] = nullptr;
          message["tool_calls"] = json::array({openai_tool_call(*tool_call, id)});
          finish_reason = "tool_calls";
        }
        json_response(
            response, 200,
            {{"id", id},
             {"object", "chat.completion"},
             {"created", std::time(nullptr)},
             {"model", response_model},
             {"choices", {{{"index", 0},
                            {"message", message},
                            {"logprobs", nullptr},
                            {"finish_reason", finish_reason}}}},
             {"usage", {{"prompt_tokens", prompt_tokens},
                        {"completion_tokens", completion_tokens},
                        {"total_tokens", prompt_tokens + completion_tokens}}}});
        return;
      }

      auto context = std::make_shared<StreamContext>();
      context->state = std::make_shared<RequestState>();
      context->engine_lock = std::move(engine_lock);
      context->engine = &engine;
      context->id = id;
      context->model = response_model;
      context->worker = std::thread([context, prompt = std::move(prompt), enable_thinking,
                                     max_tokens, sampling]() mutable {
        context->engine->run(prompt, enable_thinking, max_tokens, sampling, *context->state);
      });

      response.set_header("Cache-Control", "no-cache");
      response.set_header("X-Accel-Buffering", "no");
      response.set_chunked_content_provider(
          "text/event-stream",
          [context](size_t, httplib::DataSink& sink) {
            if (!context->sent_role) {
              context->sent_role = true;
              const std::string event = "data: " +
                  stream_chunk(context->id, context->model, {{"role", "assistant"}}, nullptr)
                      .dump() +
                  "\n\n";
              return sink.write(event.data(), event.size());
            }

            std::string text;
            bool finished = false;
            bool failed = false;
            {
              std::unique_lock<std::mutex> lock(context->state->mutex);
              context->state->changed.wait(lock, [&] {
                return !context->state->chunks.empty() || context->state->finished;
              });
              if (!context->state->chunks.empty()) {
                text = std::move(context->state->chunks.front());
                context->state->chunks.pop_front();
              }
              finished = context->state->finished && context->state->chunks.empty();
              failed = context->state->failed;
            }

            if (!text.empty()) {
              context->pending += text;
              if (!context->output_mode_decided) {
                constexpr const char* marker = "<tool_call>";
                const std::string marker_string(marker);
                const size_t first = context->pending.find_first_not_of(" \t\r\n");
                const std::string candidate =
                    first == std::string::npos ? "" : context->pending.substr(first);
                if (candidate.size() >= marker_string.size() &&
                    candidate.compare(0, marker_string.size(), marker_string) == 0) {
                  context->output_mode_decided = true;
                  context->tool_mode = true;
                } else if (candidate.size() > marker_string.size() ||
                           marker_string.compare(0, candidate.size(), candidate) != 0) {
                  context->output_mode_decided = true;
                }
              }

              if (context->output_mode_decided && !context->tool_mode) {
                const std::string event = "data: " +
                    stream_chunk(context->id, context->model,
                                 {{"content", context->pending}}, nullptr)
                        .dump() +
                    "\n\n";
                context->pending.clear();
                if (!sink.write(event.data(), event.size())) return false;
              }
            }

            if (finished && !context->sent_finish) {
              context->sent_finish = true;
              json finish_reason = failed ? json("error") : json("stop");
              if (context->tool_mode && !failed) {
                if (const auto tool_call = parse_tool_call(context->pending)) {
                  const json streamed_call =
                      {{"index", 0},
                       {"id", tool_call_id(context->id)},
                       {"type", "function"},
                       {"function",
                        {{"name", tool_call->name}, {"arguments", tool_call->arguments}}}};
                  const std::string tool_event = "data: " +
                      stream_chunk(context->id, context->model,
                                   {{"tool_calls", json::array({streamed_call})}}, nullptr)
                          .dump() +
                      "\n\n";
                  if (!sink.write(tool_event.data(), tool_event.size())) return false;
                  context->pending.clear();
                  finish_reason = "tool_calls";
                }
              }
              if (!context->pending.empty()) {
                const std::string pending_event = "data: " +
                    stream_chunk(context->id, context->model,
                                 {{"content", context->pending}}, nullptr)
                        .dump() +
                    "\n\n";
                if (!sink.write(pending_event.data(), pending_event.size())) return false;
                context->pending.clear();
              }
              const std::string final_event = "data: " +
                  stream_chunk(context->id, context->model, {{"content", ""}},
                               finish_reason)
                      .dump() +
                  "\n\ndata: [DONE]\n\n";
              sink.write(final_event.data(), final_event.size());
              sink.done();
              return false;
            }
            return true;
          },
          [context](bool success) {
            if (!success) context->engine->abort();
            if (context->worker.joinable()) context->worker.join();
            context->engine_lock.reset();
          });
    });

    server.set_exception_handler([](const auto&, auto& response, std::exception_ptr error) {
      std::string message = "internal server error";
      try {
        if (error) std::rethrow_exception(error);
      } catch (const std::exception& exception) {
        message = exception.what();
      }
      json_response(response, 500, error_body(message, "server_error"));
    });

    std::cerr << "kobold-llm: ready on 0.0.0.0:" << port << " model=" << model_name
              << std::endl;
    if (!server.listen("0.0.0.0", port)) {
      throw std::runtime_error("failed to listen on port " + std::to_string(port));
    }
    signal_server = nullptr;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "kobold-llm: fatal: " << error.what() << std::endl;
    return 1;
  }
}
