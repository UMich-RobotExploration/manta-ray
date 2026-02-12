//
// Created by tko on 2/11/26.
//

#pragma once
#include "Eigen/Dense"
#include "manif/manif.h"
#include "RobotI.h"

namespace rb {

class ConstantVelRobot : public RobotI {
public:
  const Eigen::Vector3d constantVel_;
  // @brief Constructs constant linear velocity
  explicit ConstantVelRobot(Eigen::Vector3d constantVel)
      : constantVel_(constantVel) {};
  manif::SE3Tangentd computeLocalTwist(DynamicsBodies &bodies) override;
};

} // namespace rb
