//
// Created by tko on 2/10/26.
//
#include "rb/pch.h"

#include "rb/Integrator.h"

namespace rb {
void integratePose(const manif::SE3d &pose, const manif::SE3Tangentd &twist,
                   double dt, manif::SE3d &outputPose) {
  // Using the exponential map for SE(3) to integrate the pose in body frame
  outputPose = pose.rplus(twist * dt);
  return;
}
} // namespace rb