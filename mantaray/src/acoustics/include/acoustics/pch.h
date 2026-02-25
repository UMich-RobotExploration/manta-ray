/**
 * @file pch.h
 * @brief precompiled headers for acoustics library
 */

#pragma once

// Standard library headers
#include <algorithm>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#ifndef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif

#include <spdlog/spdlog.h>
