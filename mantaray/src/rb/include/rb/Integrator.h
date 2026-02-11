//
// Created by tko on 2/10/26.
//

#pragma once

#include "manif/manif.h"

namespace rb {
typedef Eigen::Vector<double, 6> Vector6d;
typedef Eigen::Matrix<double, 6, 6> massMatrix;
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

inline void computeAccel(const manif::SE3Tangentd &twistLocalInput,
                         const massMatrix &massMatrixLocal,
                         const Vector6d &wrenchLocal,
                         Vector6d &accelerationLocal) {
  accelerationLocal = massMatrixLocal.inverse() *
                      (wrenchLocal + twistLocalInput.smallAdj().transpose() *
                                         massMatrixLocal * twistLocalInput);
};

inline void integrateVel(const massMatrix &massMatrixLocal,
                         const Vector6d &wrenchLocal,
                         const manif::SE3Tangentd &twistLocalInput, double dt,
                         manif::SE3Tangentd &twistLocalOutput,
                         Vector6d &accelerationLocalOutput) {
  // RK4 integration for twist (velocity in body frame)
  Vector6d k1, k2, k3, k4;

  // k1 = f(t, v)
  computeAccel(twistLocalInput, massMatrixLocal, wrenchLocal, k1);

  // k2 = f(t + dt/2, v + dt*k1/2)
  manif::SE3Tangentd v2(twistLocalInput.coeffs() + 0.5 * dt * k1);
  computeAccel(v2, massMatrixLocal, wrenchLocal, k2);

  // k3 = f(t + dt/2, v + dt*k2/2)
  manif::SE3Tangentd v3(twistLocalInput.coeffs() + 0.5 * dt * k2);
  computeAccel(v3, massMatrixLocal, wrenchLocal, k3);

  // k4 = f(t + dt, v + dt*k3)
  manif::SE3Tangentd v4(twistLocalInput.coeffs() + dt * k3);
  computeAccel(v4, massMatrixLocal, wrenchLocal, k4);

  // Output the RK4-weighted average acceleration
  accelerationLocalOutput = (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;

  // Final integration: v_{n+1} = v_n + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
  twistLocalOutput =
      manif::SE3Tangentd(twistLocalInput.coeffs() + (dt)*accelerationLocalOutput);
}
} // namespace rb