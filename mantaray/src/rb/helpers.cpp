//
// Created by tko on 2/18/26.
//
#include "rb/helpers.h"

namespace rb {

size_t computeNumTimeSteps(double endTimeSec, double freqHz) {
  return (static_cast<size_t>(std::round(endTimeSec * freqHz)) + 1);
}
namespace detail {
bool isEqual(double x, double y) {
  if (x == y)
    return true; // Handles exact equality, including 0.0
  double diff = std::fabs(x - y);
  double maxAbs = std::max(std::fabs(x), std::fabs(y));
  if (maxAbs < kFloatingPointToleranceNearZero) // Both near zero
    return diff < kFloatingPointToleranceDouble;
  return diff < kFloatingPointToleranceDouble * maxAbs;
}
bool validDeltaTMultiple(double time, double dt) {
  // Two cases in floating point:
  //  - Case 1: the remainder is near 0 (time is just past a multiple of dt)
  //  - Case 2: the remainder is basically dt (time is just before the next
  //    multiple; fmod can return dt - eps rather than 0)
  //
  // We use an ABSOLUTE tolerance (kDeltaTMultipleTolerance) rather than one
  // scaled by dt, because the residual we're guarding against comes from
  // long-horizon accumulation of a non-binary-exact dt (e.g. 0.1) — its
  // magnitude grows with step count, not with dt. See TODO.md for the
  // architectural fix (integer sim clock).
  double timeStepRemainder = std::fmod(time, dt);
  bool hasNoRemainder = timeStepRemainder < kDeltaTMultipleTolerance;
  bool nearDt =
      std::fabs(std::fabs(dt) - timeStepRemainder) < kDeltaTMultipleTolerance;
  return (hasNoRemainder || nearDt);
}
} // namespace detail
} // namespace rb