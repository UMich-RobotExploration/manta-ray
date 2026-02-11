//
// Created by tko on 2/11/26.
//

#pragma once

#include "manif/manif.h"
#include "rb/PhysicsBodies.h"

namespace rb {
class RobotI {
public:
  explicit RobotI(DynamicsBodies& bodies);
  BodyIdx bodyIdx_{0};
  bool isValid_{false};
  virtual manif::SE3Tangentd computeLocalTwist(DynamicsBodies &bodies);
};

}
