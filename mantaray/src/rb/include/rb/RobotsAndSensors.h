/**
 * @file RobotsAndSensors.h
 * @brief Houses the current concrete implementations of sensors and robots.
 *
 * @details For colocation, new *concrete* types should be implemented here
 * until this file become too unwieldy.
 */

#pragma once
#include "Eigen/Dense"
#include "manif/manif.h"
#include <fstream>
#include <random>

#include "RbInterfaces.h"
#include "mantaray/utils/checkAssert.h"
#include "rb/helpers.h"

namespace rb {

////////////////////////////////////////////////////////////////////////////////
/// Robot Implementations
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Configuration for ConstantVelRobot
 * @param position initial position
 * @param velocity constant velocity robot will follow
 */
struct ConstantVelConfig {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
};

/**
 * @brief Constant Velocity Robot in prescribed vector direction
 */
class ConstantVelRobot : public RobotI {
public:
  const Eigen::Vector3d constantVel_;
  explicit ConstantVelRobot(const ConstantVelConfig &cfg)
      : constantVel_(cfg.velocity){};
  manif::SE3Tangentd computeLocalTwist(const DynamicsBodies &bodies,
                                       double simTime, double dt) override;
};

////////////////////////////////////////////////////////////////////////////////
/// Sensor Implementations
////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Odometry in the X-Y plane of the vehicle. No Z - noise
 */
class PositionalXYOdometry : public SensorI {
public:
  PositionalXYOdometry(double freqHz, int timeSteps,
                       std::normal_distribution<double>);
  const std::vector<Eigen::VectorXd> &getSensorData() override;
  const std::vector<double> &getSensorTimesteps() override;
  void updateSensor(const DynamicsBodies &bodies, double simTime,
                    std::mt19937 &rngEngine) override;

  double getNoiseStddev() const { return noiseDist_.stddev(); }

private:
  std::normal_distribution<double> noiseDist_;
  Eigen::Vector3d prevPosition_{0.0, 0.0, 0.0};
};

/**
 * @brief Sensor for extracting ground truth pose data from simulation
 * @details Ignores rngEngine, and strictly pulls in flattened SE3 pose for
 * each requested timestep. The pose in SE3 is represented as
 *  \f[
 *  x_{pose} =
 *  \begin{bmatrix}
 *  x & y & z & q_1 & q_2 & q_3 & q_4
 *  \end{bmatrix}
 *  \f]
 */
class GroundTruthPose : public SensorI {
public:
  GroundTruthPose(double freqHz, int timeSteps);
  const std::vector<Eigen::VectorXd> &getSensorData() override;
  const std::vector<double> &getSensorTimesteps() override;
  void updateSensor(const DynamicsBodies &bodies, double simTime,
                    std::mt19937 &rngEngine) override;
};

/**
 * @brief GPS-like position sensor.
 *
 * @details Only produces a measurement when the robot is within
 * `surfaceRangeMeters` of the surface (|z - surfaceDepth| <= surfaceRange).
 * Noise is anisotropic: z-noise is larger than x/y.
 */
class GpsPosition : public SensorI {
public:
  GpsPosition(double freqHz, int timeSteps,
              std::normal_distribution<double> xyNoiseDist,
              std::normal_distribution<double> zNoiseDist,
              double surfaceRangeMeters = 0.1, double surfaceDepth = 0.0);

  const std::vector<Eigen::VectorXd> &getSensorData() override;
  const std::vector<double> &getSensorTimesteps() override;

  void updateSensor(const DynamicsBodies &bodies, double simTime,
                    std::mt19937 &rngEngine) override;

  double getXyNoiseStddev() const { return xyNoiseDist_.stddev(); }
  double getZNoiseStddev() const { return zNoiseDist_.stddev(); }

private:
  std::normal_distribution<double> xyNoiseDist_;
  std::normal_distribution<double> zNoiseDist_;
  double surfaceRangeMeters_{0.1};
  double surfaceDepth_{0.0};
};

////////////////////////////////////////////////////////////////////////////////
/// Dumping methods
////////////////////////////////////////////////////////////////////////////////

// These methods need to be updated when adding a new SensorType
// Errors are provided if they are not updated
const char *csvHeaderForSensor(const SensorType type);
std::string csvRowForSensor(const SensorType type, const Eigen::VectorXd data);

void outputRobotSensorToCsv(const char *baseFilename, const RobotI &robot);
} // namespace rb
