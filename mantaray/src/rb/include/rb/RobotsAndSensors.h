// @brief Houses the current concrete implementations of sensors and robots.
//
//

#pragma once
#include "Eigen/Dense"
#include "RbInterfaces.h"
#include "manif/manif.h"

namespace rb {

class ConstantVelRobot : public RobotI {
public:
  const Eigen::Vector3d constantVel_;
  // @brief Constructs constant linear velocity
  explicit ConstantVelRobot(Eigen::Vector3d constantVel)
      : constantVel_(constantVel){};
  manif::SE3Tangentd computeLocalTwist(const DynamicsBodies &bodies) override;
};

class PositionalOdomoetry : public SensorI {
public:
  PositionalOdomoetry(double freqHz, double timeSteps);
  std::vector<Eigen::VectorXd> getSensorData() override;
  std::vector<double> getSensorTimesteps() override;
  void updateSensor(const DynamicsBodies &bodies, double simTime) override;

private:
};
} // namespace rb
