//
// Created by tko on 2/10/26.
//
#include "rb/pch.h"

#include "rb/Integrator.h"

namespace rb {
// inline void integratePose(const manif::SE3d &poseSpatial, const manif::SE3Tangentd &twistLocal,
//                    double dt, manif::SE3d &outputPose) {
//   // Using the exponential map for SE(3) to integrate the pose in body frame
//   // Equation 25: Micro Lie Theory
//   outputPose = poseSpatial.rplus(twistLocal * dt);
//   return;
// }
} // namespace rb