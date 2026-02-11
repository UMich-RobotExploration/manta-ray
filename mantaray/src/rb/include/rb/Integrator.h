//
// Created by tko on 2/10/26.
//

#pragma once

#include "manif/manif.h"

namespace rb {
typedef Eigen::Vector<double, 6> Vector6d;
/*
 * @brief Integrates the pose of rigid body given twist in body frame
 */
inline void integratePose(const manif::SE3d &poseSpatial,
                          const manif::SE3Tangentd &twistLocal, double dt,
                          manif::SE3d &outputPose) {

  // Using the exponential map for SE(3) to integrate the pose in body frame //
  // Equation 25: Micro Lie Theory
  outputPose = poseSpatial.rplus(twistLocal * dt);
  return;
};
} // namespace rb