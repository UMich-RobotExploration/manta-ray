/**
 * @file helpers.h
 * @brief Few critical helpers for floating point comparison.
 */

#pragma once

namespace rb {
constexpr double kBoundaryEpsilonDouble =
    std::numeric_limits<double>::epsilon() * 100;
constexpr double kFloatingPointToleranceDouble = 1e-10;
constexpr double kFloatingPointToleranceNearZero = 1e-12;
size_t computeNumTimeSteps(double endTimeSec, double freqHz);
namespace detail {
bool isEqual(double x, double y);

/**
 * @brief Checks to see if time is a valid frequency = 1/dt multiple.
 * @details Handles floating point complexity
 * - Evaluates 2 cases:
 *  - A remainder near zero,
 *  - A remainder near dt.
 */
bool validDeltaTMultiple(double time, double dt);

} // namespace detail
} // namespace rb