//
// Created by tko on 2/11/26.
//

#pragma once
#include "PhysicsBodies.h"
#include "RobotI.h"
#include "Integrator.h"

namespace rb {
/**
 * @para Why World?
 * World exists to house the DynamicBodies and robots together. Robots
 * store indexes into DynamicsBodies, so this way we can ensure
 * there are no null references,and we can manage the lifecycle of the dynamics
 * bodies and robots together
 *
 */
typedef size_t RobotIdx;
struct RbWorld {
  DynamicsBodies dynamicsBodies;
  std::vector<Eigen::Vector3d> landmarks;
  std::vector<std::unique_ptr<RobotI>> robots;
};

// Factory that constructs robots, but I return no dangling pointers
// Access them through the index
template<typename RobotType, typename... Args>
RobotIdx addRobot(RbWorld &world, Args&&... args) {
  // First, add a body to get the index
  BodyIdx newIdx = addDynamicsBody(world.dynamicsBodies);

  // Create robot and immediately set its index
  auto robot = std::make_unique<RobotType>(std::forward<Args>(args)...);
  robot->setDynamicsIndex(newIdx);

  // Store and return reference
  RobotType* ptr = robot.get();
  world.robots.push_back(std::move(robot));
  return static_cast<RobotIdx>(newIdx);
}

void reserveRobots(RbWorld &world, size_t count);
void reserveLandmarks(RbWorld &world, size_t count);
void addLandmark(RbWorld &world, const Eigen::Vector3d &landmark);

/*
 * @brief Gathers twists and the integrates them
 */
void stepWorld(RbWorld &world, double dt);

} // namespace rb