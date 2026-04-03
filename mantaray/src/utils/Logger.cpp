// Logger.cpp
#include "mantaray/utils/Logger.h"
#include <filesystem>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

std::shared_ptr<spdlog::logger> global_logger;
std::shared_ptr<spdlog::logger> bellhop_logger;

void init_logger(const std::string &logDir) {
  auto logPath = std::filesystem::path(logDir);

  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
      (logPath / "sim_log.txt").string(), true);
  console_sink->set_level(spdlog::level::debug);
  // [7 width centered warning level][filename:line number] log message
  console_sink->set_pattern("%^[%=7l][%s:%#]%$ %v");
  file_sink->set_pattern("[%=7l] %v");

  global_logger = std::make_shared<spdlog::logger>(
      "global_logger", spdlog::sinks_init_list{console_sink, file_sink});

  spdlog::set_default_logger(global_logger);
  spdlog::set_level(spdlog::level::debug);

  auto bellhop_file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
      (logPath / "sim_bellhop_log.txt").string(), false);
  bellhop_logger = std::make_shared<spdlog::logger>(
      "bellhop_logger", spdlog::sinks_init_list{bellhop_file_sink});
  // removing any newlines with custom pattern for logger
  auto f = std::make_unique<spdlog::pattern_formatter>(
      "%v", spdlog::pattern_time_type::local, std::string("")); // disable eol
  bellhop_file_sink->set_formatter(std::move(f));
  bellhop_logger->set_level(spdlog::level::debug);
}
