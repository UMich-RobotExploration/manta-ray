//
// Created by tko on 2/11/26.
//

#include "rb/pch.h"

#include "include/rb/ConstantVelRobot.h"

namespace rb {

manif::SE3Tangentd
ConstantVelRobot::computeLocalTwist(DynamicsBodies &bodies,
                                    Eigen::Vector3d constantVel) {
  // For this simple float, I test just constant velocity right now
  return manif::SE3Tangentd(constantVel_, Eigen::Vector3d(0.0, 0.0, 0.0));
}

} // namespace rb