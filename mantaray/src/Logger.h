// Logger.h
#pragma once
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG

#include <spdlog/pattern_formatter.h>
#include <spdlog/spdlog.h>

// Global logger for normal use
extern std::shared_ptr<spdlog::logger> global_logger;

// Bellhop callback only logger (not for general use)
extern std::shared_ptr<spdlog::logger> bellhop_logger;

/*
 * @brief Initializes both the global logger and bellhop logger
 * @details
 * - Global logger is for all generic logging tasks
 * - Bellhop logger is there to dump the LARGE Bellhop callback logs. It is
 *    specifically formatted to support the text dumps so it is not good for
 * normal log use. It should only receive logs from the bellhop callbacks. It
 * does not stream to stdout
 */
void init_logger();
