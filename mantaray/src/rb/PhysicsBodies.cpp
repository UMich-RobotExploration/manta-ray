//
// Created by tko on 2/9/26.
//

#include "rb/pch.h"

#include "checkAssert.h"
#include "rb/Integrator.h"
#include "rb/PhysicsBodies.h"

namespace rb {

void DynamicsBodies::reserveBodies(size_t count) { kinematics.reserve(count); }

const KinematicData &DynamicsBodies::getKinematicData(BodyIdx index) const {
  CHECK(index < kinematics.size(),
        "Index out of bounds for dynamics properties");
  return kinematics[index];
}
KinematicData &DynamicsBodies::getKinematicData(BodyIdx index) {
  CHECK(index < kinematics.size(),
        "Index out of bounds for dynamics properties");
  return kinematics[index];
}
Eigen::Vector3d DynamicsBodies::getLinearVelocity(BodyIdx index) const {
  CHECK(index < kinematics.size(),
        "Index out of bounds for dynamics properties");
  return kinematics[index].twistLocal.lin();
}
Eigen::Vector3d DynamicsBodies::getPosition(BodyIdx index) const {
  CHECK(index < kinematics.size(),
        "Index out of bounds for dynamics properties");
  return kinematics[index].poseGlobal.translation();
}
void DynamicsBodies::setPose(BodyIdx index, const Eigen::Vector3d &position,
                             const Eigen::Quaterniond &orientation) {
  setPosition(index, position);
  setOrientation(index, orientation);
}
void DynamicsBodies::setPosition(BodyIdx index,
                                 const Eigen::Vector3d &position) {
  kinematics[index].poseGlobal.coeffs().block<3, 1>(0, 0) = position;
}
void DynamicsBodies::setOrientation(BodyIdx index,
                                    const Eigen::Quaterniond &orientation) {
  kinematics[index].poseGlobal.coeffs().block<4, 1>(3, 0) =
      orientation.coeffs();
}

void relativeTransform(const manif::SE3d &pose, const manif::SE3d &refPose,
                       manif::SE3d &outputPose) {
  outputPose = pose * refPose.inverse();
}

namespace detail {
BodyIdx addDynamicsBody(DynamicsBodies &bodies) {
  BodyIdx newIndex = bodies.kinematics.size();
  bodies.kinematics.emplace_back(); // Default construct kinematic data
  return newIndex;
}
} // namespace detail

} // namespace rb