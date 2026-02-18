//
// Created by tko on 2/18/26.
//
#include "rb/helpers.h"

namespace rb {

namespace detail {
bool isEqual(double x, double y) {
  return std::fabs(x - y) < kBoundaryEpsilonDouble;
}
bool validDeltaTMultiple(double time, double dt) {
  // Basically two cases with floating points
  // - Case 1: The remainder is near 0
  // - Case 2: The remainder is basically dt
  // True when no remainder, false when remainder
  double timeStepRemainder = std::fmod(time, dt);
  bool hasNoRemainder = timeStepRemainder < kBoundaryEpsilonDouble;
  // True when dt, false when not dt
  bool isEqualTodt = isEqual(timeStepRemainder, dt);
  return (hasNoRemainder || isEqualTodt);
}
} // namespace detail
} // namespace rb