// Houses the current concrete implementations of sensors and robots.

#pragma once
#include "Eigen/Dense"
#include "manif/manif.h"
#include <fstream>
#include <random>

#include "RbInterfaces.h"
#include "checkAssert.h"
#include "rb/helpers.h"

namespace rb {

class ConstantVelRobot : public RobotI {
public:
  const Eigen::Vector3d constantVel_;
  /** @brief Constructs constant linear velocity
   */
  explicit ConstantVelRobot(Eigen::Vector3d constantVel)
      : constantVel_(constantVel){};
  manif::SE3Tangentd computeLocalTwist(const DynamicsBodies &bodies) override;
};

class PositionalXYOdomoetry : public SensorI {
public:
  PositionalXYOdomoetry(double freqHz, int timeSteps,
                        std::normal_distribution<double>);
  const std::vector<Eigen::VectorXd> &getSensorData() override;
  const std::vector<double> &getSensorTimesteps() override;
  void updateSensor(const DynamicsBodies &bodies, double simTime,
                    std::mt19937 &rngEngine) override;

private:
  std::normal_distribution<double> noiseDist_;
  Eigen::Vector3d prevPosition_{0.0, 0.0, 0.0};
};
class GroundTruthPose : public SensorI {
public:
  GroundTruthPose(double freqHz, int timeSteps);
  const std::vector<Eigen::VectorXd> &getSensorData() override;
  const std::vector<double> &getSensorTimesteps() override;
  void updateSensor(const DynamicsBodies &bodies, double simTime,
                    std::mt19937 &rngEngine) override;
};

// These methods need to be updated when adding a new SensorType
// Errors are provided if they are not updated

const char *csvHeaderForSensor(const SensorType type);
std::string csvRowForSensor(const SensorType type, const Eigen::VectorXd data);

void outputRobotSensorToCsv(const char *baseFilename, const RobotI &robot);
} // namespace rb
