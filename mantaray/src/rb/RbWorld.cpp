//
// Created by tko on 2/11/26.
//

#include "rb/pch.h"

#include "rb/RbWorld.h"

namespace rb {
void reserveRobots(RbWorld &world, size_t count) {
  world.robots.reserve(count);
  reserveBodies(world.dynamicsBodies, count);
}
void reserveLandmarks(RbWorld &world, size_t count) {
  world.landmarks.reserve(count);
}
void addLandmark(RbWorld &world, const Eigen::Vector3d &landmark) {
  world.landmarks.emplace_back(landmark);
}

void gatherTwists(RbWorld &world) {
  for (auto &robot : world.robots) {
    auto twist = robot->computeLocalTwist(world.dynamicsBodies);
    getKinematicData(world.dynamicsBodies, robot->bodyIdx_).twistLocal = twist;
  }
}

void stepWorld(RbWorld &world, double dt) {
  gatherTwists(world);
  for (auto &kinematicData : world.dynamicsBodies.kinematics) {
    manif::SE3d newPose;
    integratePose(kinematicData.poseGlobal, kinematicData.twistLocal, dt,
                  newPose);
    kinematicData.poseGlobal = newPose;
  }
}
} // namespace rb
