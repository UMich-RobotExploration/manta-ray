// Logger.h
#pragma once
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG

#include <spdlog/pattern_formatter.h>
#include <spdlog/spdlog.h>

extern std::shared_ptr<spdlog::logger> global_logger;
extern std::shared_ptr<spdlog::logger> bellhop_logger;

void init_logger();
