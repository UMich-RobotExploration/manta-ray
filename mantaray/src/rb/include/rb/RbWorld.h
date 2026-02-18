//
// Created by tko on 2/11/26.
//

#pragma once
#include <random>

#include "Integrator.h"
#include "PhysicsBodies.h"
#include "checkAssert.h"
#include "rb/RbInterfaces.h"
#include "rb/helpers.h"

namespace rb {

typedef size_t RobotIdx;
/**
 * @par Why World?
 * World exists to house the DynamicBodies and robots together. Robots
 * store indexes into DynamicsBodies, so this way we can ensure
 * there are no null references,and we can manage the lifecycle of the dynamics
 * bodies and robots together
 *
 * @par Construction
 * To construct RbWorld, initialize use default construction
 * and then add robots and landmarks as you see fit.
 * The only requirement is createRngEngine must be called before advancing the
 * world to ensure the rng engine is initialized.
 */
struct RbWorld {
  DynamicsBodies dynamicsBodies;
  SimData simData{0.0, 0.1};
  std::vector<Eigen::Vector3d> landmarks;
  std::vector<std::unique_ptr<RobotI>> robots;
  std::unique_ptr<std::mt19937> rngEngine;

  void reserveRobots(size_t count);
  void reserveLandmarks(size_t count);
  void addLandmark(const Eigen::Vector3d &landmark);
  void validateWorld();

  /*
   * @brief Gathers twists and the integrates them
   */
  void stepWorld(double dt);

  void advanceWorld(double time);

  // @brief Create a random generator that works for entirety of sim
  void createRngEngine(size_t seed);

  // Factory that constructs robots, but I return no dangling pointers
  // Access them through the index
  template <typename RobotType, typename... Args>
  RobotIdx addRobot(Args &&...args) {
    // First, add a body to get the index
    BodyIdx newIdx = detail::addDynamicsBody(dynamicsBodies);

    // Create robot and immediately set its index
    auto robot = std::make_unique<RobotType>(std::forward<Args>(args)...);
    robot->setDynamicsIndex(newIdx);

    // Store and return reference
    robots.push_back(std::move(robot));
    return static_cast<RobotIdx>(newIdx);
  }
};

} // namespace rb