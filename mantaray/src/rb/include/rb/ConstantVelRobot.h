//
// Created by tko on 2/11/26.
//

#pragma once
#include "Eigen/Dense"
#include "manif/manif.h"
#include "RobotI.h"

namespace rb {

class ConstantVelRobot : RobotI {
public:
  const Eigen::Vector3d constantVel_;
  explicit ConstantVelRobot(DynamicsBodies &bodies, Eigen::Vector3d constantVel)
      : RobotI(bodies), constantVel_(constantVel) {};
  manif::SE3Tangentd computeLocalTwist(DynamicsBodies &bodies) override;
};

} // namespace rb
