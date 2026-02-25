/** @file RbWorld.h
 *
 * @brief Controls all rigid body simulation time stepping
 *
 * @details Manages all the simulation time stepping along with validation. If
 * there are preconditions or similar that are needed for the simulation they
 * should be directly included in @ref RbWorld::validateWorld.
 *
 * @see RbWorld::validateWorld
 */
#pragma once
#include <random>

#include "Integrator.h"
#include "PhysicsBodies.h"
#include "checkAssert.h"
#include "rb/RbInterfaces.h"
#include "rb/RobotsAndSensors.h"
#include "rb/helpers.h"

/** @brief Namespace handling all rigid body code */
namespace rb {

typedef size_t RobotIdx;

/**
 * @brief Owner of all rigid body dynamics values
 * @details Struct provides access to values and ensures lifetimes
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
  /**
   * @brief Ensure all preconditions are met to begin simulation
   *
   * @details
   * Checks the following:
   *  - Appropriate dt and sensor frequencies
   *  - Initialization of rng engine
   *  - Proper allocation of sensor data and timestamp vectors
   *  - Ensure SensorType has been populated properly
   */
  void validateWorld();

  /**
   * @brief Advances the world dt seconds
   * @details *UPDATES* Sim time. It is the sole updated of sim time.
   */
  void stepWorld(double dt);

  /**
   * @brief Advances the world to the specified absolute time
   * @param time desired time to advance to (absolute)
   * @throws runtime_error if request is backwards in time
   *
   * @par Important Items:
   * When the time requested is not an integer multiple of dt, the advance world
   * function allows for and corrects for this non-uniform step.
   * The *reason* this is done, is to enable specific advancements requested
   * by the acoustic sim if needed.
   */
  void advanceWorld(double time);

  /** @brief Create a random generator that works for entirety of sim
   */
  void createRngEngine(size_t seed);

  /** @brief Factory that constructs robots, but returns no dangling pointers
   * Access them through the index
   */
  template <typename RobotType, typename... Args>
  RobotIdx addRobot(Args &&...args) {
    // First, add a body to get the index
    BodyIdx newIdx = detail::addDynamicsBody(dynamicsBodies);

    // Create robot and immediately set its index
    auto robot = std::make_unique<RobotType>(std::forward<Args>(args)...);
    robot->setDynamicsIndex(newIdx);

    // Store and return reference
    robots.push_back(std::move(robot));
    spdlog::info("Adding robot to world at index {}", newIdx);

    return static_cast<RobotIdx>(newIdx);
  }
};

} // namespace rb