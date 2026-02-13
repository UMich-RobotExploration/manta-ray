//
// Created by tko on 2/9/26.
//

#include "rb/pch.h"

#include "checkAssert.h"
#include "rb/Integrator.h"
#include "rb/PhysicsBodies.h"

namespace rb {

void DynamicsBodies::reserveBodies(size_t count) { kinematics.reserve(count); }

KinematicData &DynamicsBodies::getKinematicData(BodyIdx index) {
  CHECK(index < kinematics.size(),
        "Index out of bounds for dynamics properties");
  return kinematics[index];
}
Eigen::Vector3d DynamicsBodies::getPosition(BodyIdx index) {
  CHECK(index < kinematics.size(),
                "Index out of bounds for dynamics properties");
        return kinematics[index].poseGlobal.translation();
}

void relativeTransform(manif::SE3d &pose, manif::SE3d &refPose,
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