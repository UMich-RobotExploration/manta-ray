//
// Created by tko on 2/11/26.
//

#include "rb/pch.h"

#include "rb/RbWorld.h"

namespace rb {
void RbWorld::reserveRobots(size_t count) {
  robots.reserve(count);
  dynamicsBodies.reserveBodies(count);
}
void RbWorld::reserveLandmarks(size_t count) { landmarks.reserve(count); }
void RbWorld::addLandmark(const Eigen::Vector3d &landmark) {
  landmarks.emplace_back(landmark);
}

/*
 * @brief Gathers the local twists from each robot and stores them in the bodies
 * @details Sets all dead robots' twist to zeros. Prevents conditionals in
 * update loop
 */
void gatherTwists(RbWorld &world) {
  for (auto &robot : world.robots) {
    if (robot->isAlive_) {
      auto twist = robot->computeLocalTwist(world.dynamicsBodies);
      world.dynamicsBodies.getKinematicData(robot->bodyIdx_).twistLocal = twist;
    } else {
      world.dynamicsBodies.getKinematicData(robot->bodyIdx_)
          .twistLocal.setZero();
    }
  }
}

void RbWorld::stepWorld(double dt) {
  gatherTwists(*this);
  for (auto &kinematicData : dynamicsBodies.kinematics) {
    manif::SE3d newPose;
    integratePose(kinematicData.poseGlobal, kinematicData.twistLocal, dt,
                  newPose);
    kinematicData.poseGlobal = newPose;
  }
  simData.time += dt;
}

/**
 * @brief Advances the world to the specified absolute time
 * @param time desired time to advance to (absolute)
 * @throws runtime_error if request is backwards in time
 */
void RbWorld::advanceWorld(double time) {
  if (detail::isEqual(time, simData.time)) {
    return; // No advancement needed
  } else if (time < simData.time) {
    std::string msg = "Cannot simulate the requested step as it is backwards "
                      "in time. Current time: " +
                      std::to_string(simData.time) +
                      ", requested time: " + std::to_string(time);
    throw std::runtime_error(msg);
  } else {
    double timeToAdvance = time - simData.time;
    while (timeToAdvance > 0) {
      double dt = std::min(simData.dt, timeToAdvance);
      stepWorld(dt);
      timeToAdvance -= dt;
    }
    return;
  }
}

namespace detail {
bool isEqual(double x, double y) {
  return std::abs(x - y) < kBoundaryEpsilonDouble;
}
} // namespace detail

} // namespace rb
