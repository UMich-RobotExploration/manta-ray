//
// Created by tko on 2/11/26.
//

#pragma once

#include "manif/manif.h"
#include "rb/PhysicsBodies.h"

namespace rb {
class RobotI {
public:
  virtual ~RobotI();  // Add virtual destructor to ensure proper cleanup

  void setDynamicsIndex(BodyIdx idx);
  BodyIdx getBodyIdx() const { return bodyIdx_; }

  virtual manif::SE3Tangentd computeLocalTwist(DynamicsBodies &bodies) = 0;  // Make pure virtual

  BodyIdx bodyIdx_{0};
  bool isValid_{false};
};

}
