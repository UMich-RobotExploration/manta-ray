// Logger.cpp
#include "Logger.h"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

std::shared_ptr<spdlog::logger> global_logger;
std::shared_ptr<spdlog::logger> bellhop_logger;

void init_logger() {
  // Create a console and file sink
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  auto file_sink =
      std::make_shared<spdlog::sinks::basic_file_sink_mt>("sim_log.txt", true);
  console_sink->set_level(spdlog::level::debug);
  // [7 width centered warning level][filename:line number] log message
  console_sink->set_pattern("%^[%=7l][%s:%#]%$ %v");
  file_sink->set_pattern("[%=7l] %v");

  // Create the global logger
  global_logger = std::make_shared<spdlog::logger>(
      "global_logger", spdlog::sinks_init_list{console_sink, file_sink});

  // Set the global logger as the default logger
  spdlog::set_default_logger(global_logger);
  spdlog::set_level(spdlog::level::debug); // Set log level

  auto bellhop_file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
      "sim_bellhop_log.txt", false);
  bellhop_logger = std::make_shared<spdlog::logger>(
      "bellhop_logger", spdlog::sinks_init_list{bellhop_file_sink});
  // removing any newlines with custom pattern for logger
  auto f = std::make_unique<spdlog::pattern_formatter>(
      "%v", spdlog::pattern_time_type::local, std::string("")); // disable eol
  bellhop_file_sink->set_formatter(std::move(f));
  bellhop_logger->set_level(spdlog::level::debug);
}
