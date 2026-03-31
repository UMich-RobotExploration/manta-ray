/** @file RobotFactory.h
 * @brief Convenience functions for creating robots with standard sensor suites
 */

#pragma once

#include "rb/RbWorld.h"
#include "rb/RobotsAndSensors.h"
#include "rb/helpers.h"
#include <Eigen/Core>
#include <random>

namespace sim {

/**
 * @brief Default sensor frequencies and noise parameters for the standard
 * sensor suite (GroundTruthPose, PositionalXYOdometry, GpsPosition).
 */
struct StandardSensorConfig {
  /// Ground truth pose sample rate
  double gtFreqHz{0.01};
  /// Odometry sample rate
  double odomFreqHz{0.01};
  /// GPS sample rate
  double gpsFreqHz{0.5};
  /// Odometry XY noise standard deviation
  double odomNoiseStddev{0.01};
  /// GPS XY noise standard deviation
  double gpsXyNoiseStddev{0.5};
  /// GPS Z noise standard deviation
  double gpsZNoiseStddev{2.0};
};

/**
 * @brief Creates a robot, attaches the standard sensor suite, and sets its
 * initial position.
 *
 * @details Every robot in the simulation carries the same three sensors:
 * GroundTruthPose, PositionalXYOdometry, and GpsPosition. This function
 * encapsulates that boilerplate so that adding a robot is a single call.
 *
 * @tparam RobotType The concrete robot class (e.g. rb::ConstantVelRobot)
 * @tparam Args Constructor argument types forwarded to the robot
 * @param world The simulation world to add the robot to
 * @param endTime Simulation end time (used to pre-allocate sensor buffers)
 * @param initialPos Initial 3D position in world coordinates
 * @param sensorCfg Sensor frequencies and noise parameters
 * @param robotArgs Arguments forwarded to the RobotType constructor
 * @return Index of the newly created robot
 */
template <typename RobotType, typename... Args>
rb::RobotIdx addStandardRobot(rb::RbWorld &world, double endTime,
                              const Eigen::Vector3d &initialPos,
                              const StandardSensorConfig &sensorCfg,
                              Args &&...robotArgs) {
  auto idx = world.addRobot<RobotType>(std::forward<Args>(robotArgs)...);

  world.robots[idx]->addSensor(std::make_unique<rb::GroundTruthPose>(
      sensorCfg.gtFreqHz,
      rb::computeNumTimeSteps(endTime, sensorCfg.gtFreqHz)));

  world.robots[idx]->addSensor(std::make_unique<rb::PositionalXYOdometry>(
      sensorCfg.odomFreqHz,
      rb::computeNumTimeSteps(endTime, sensorCfg.odomFreqHz),
      std::normal_distribution<double>{0.0, sensorCfg.odomNoiseStddev}));

  world.robots[idx]->addSensor(std::make_unique<rb::GpsPosition>(
      sensorCfg.gpsFreqHz,
      rb::computeNumTimeSteps(endTime, sensorCfg.gpsFreqHz),
      std::normal_distribution<double>{0.0, sensorCfg.gpsXyNoiseStddev},
      std::normal_distribution<double>{0.0, sensorCfg.gpsZNoiseStddev}));

  world.dynamicsBodies.setPosition(idx, initialPos);

  return idx;
}

} // namespace sim
