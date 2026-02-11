//
// Created by tko on 2/11/26.
//

#include "rb/pch.h"

#include "include/rb/ConstantVelRobot.h"

namespace rb {

manif::SE3Tangentd ConstantVelRobot::computeLocalTwist(DynamicsBodies &bodies) {
  // For this simple float, I test just constant velocity right now
  auto twist = manif::SE3Tangentd().setZero();
  twist.lin() = constantVel_;
  return twist;
}

} // namespace rb