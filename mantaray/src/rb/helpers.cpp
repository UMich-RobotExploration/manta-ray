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
  // Basically two cases with floating points
  // - Case 1: The remainder is near 0
  // - Case 2: The remainder is basically dt
  // True when no remainder, false when remainder
  double timeStepRemainder = std::fmod(time, dt);
  bool hasNoRemainder =
      timeStepRemainder < kFloatingPointToleranceDouble * std::fabs(dt);
  // True when dt, false when not dt
  bool isEqualTodt = isEqual(timeStepRemainder, dt);
  return (hasNoRemainder || isEqualTodt);
}
} // namespace detail
} // namespace rb