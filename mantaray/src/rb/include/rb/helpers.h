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

// Tolerance for the sim-clock "is time a multiple of dt?" check. Absolute,
// not scaled by dt: sim-clock drift is driven by accumulated summation of a
// non-binary-exact dt (e.g. 0.1), so the residual grows with step count
// rather than with dt magnitude. 1e-9 s of slack is physically negligible
// (~1000x below any sensor cadence) while covering ~1e-11 s of accumulated
// drift over multi-day sims at 0.1 s dt.
constexpr double kDeltaTMultipleTolerance = 1e-9;

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