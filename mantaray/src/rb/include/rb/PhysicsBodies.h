//
// Created by tko on 2/9/26.
//

#pragma once

#include "manif/manif.h"

namespace rb {
// Typedef for 6D Vector (twist, wrench, and combined accelerations)
typedef Eigen::Matrix<double, 6, 1> Vector6d;

typedef size_t BodyIdx;

/**
 * @brief Data necessary to integrate the state, indexed through BodyIdx
 *
 * @details Implemented as a SoA with growable vectors
 *
 */
struct KinematicData {
  manif::SE3d poseGlobal = manif::SE3d::Identity();
  manif::SE3Tangentd twistLocal = manif::SE3Tangentd::Zero();
};

struct DynamicsBodies {
  std::vector<KinematicData> kinematics;

  /** @brief Returns const kinematic data reference*/
  const KinematicData &getKinematicData(BodyIdx index) const;

  /** @brief Returns mutatable kinematic data reference*/
  KinematicData &getKinematicData(BodyIdx index);

  /** @brief Returns copy of linear velocity of robot*/
  Eigen::Vector3d getLinearVelocity(BodyIdx index) const;

  /** @brief Returns copy of linear position of robot*/
  Eigen::Vector3d getPosition(BodyIdx index) const;

  /** @brief Overwrites pose information of robot in sim
   *
   * @param index body index
   * @param position 3x1 position vector
   * @param orientation quaternion pose
   */
  void setPose(BodyIdx index, const Eigen::Vector3d &position,
               const Eigen::Quaterniond &orientation);

  /** @brief Overwrites position information of robot in sim
   *
   * @param index body index
   * @param position 3x1 position vector
   */
  void setPosition(BodyIdx index, const Eigen::Vector3d &position);

  void setOrientation(BodyIdx index, const Eigen::Quaterniond &orientation);
  void reserveBodies(size_t count);
};

/**
 * @brief Computes the relative transform of pose with respect to refPose
 * Poses must be relative to the same frame otherwise this is meaningless.
 *
 * @par Math:
 * Reference: Modern Robotics, Section 3.3.1.2, "Changing the reference frame"
 * Normally we are given the equation as follows:
 * \f[
 *   T_{ac} = T_{ab} \cdot T_{bc}
 * \f]
 * Where b is canceled, and we get T_ac as the result. However, we want the
 * relative transform. If we treat pose as T_ac and refPose as T_bc, we can see
 * we want to solve for T_ab. We can do this by:
 * \f[
 *   T_{ac} * T_{bc}^{-1} = T_{ab}
 * \f]
 *
 * @param pose Pose we want to express relative to refPose
 * @param refPose "Fixed" pose we are computing relative to
 * @param outputPose Output pose expressed in refPose frame
 */
void relativeTransform(const manif::SE3d &pose, const manif::SE3d &refPose,
                       manif::SE3d &outputPose);

/* @brief Details namespace is for private internal use of the rb library
 */
namespace detail {
/*
 * @brief Function utilized by RbWorld to add bodies. Don't use unless you
 * understand how the robots and world are actually built together!
 */
BodyIdx addDynamicsBody(DynamicsBodies &bodies);
} // namespace detail
} // namespace rb