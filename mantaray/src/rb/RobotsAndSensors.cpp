//
// Created by tko on 2/11/26.
//

#include "rb/pch.h"

#include "include/rb/RobotsAndSensors.h"

namespace rb {

manif::SE3Tangentd
ConstantVelRobot::computeLocalTwist(const DynamicsBodies &bodies) {
  // For this simple float, I test just constant velocity right now
  auto twist = manif::SE3Tangentd().setZero();
  twist.lin() = constantVel_;
  return twist;
}

PositionalOdomoetry::PositionalOdomoetry(double freqHz, double timeSteps)
    : SensorI(timeSteps, freqHz) {
  if (freqHz_ < 0) {
    throw std::invalid_argument("Frequency must be non-negative");
  }
}
std::vector<Eigen::VectorXd> PositionalOdomoetry::getSensorData() {
  return data_;
}

std::vector<double> PositionalOdomoetry::getSensorTimesteps() {
  return timesteps_;
}

void PositionalOdomoetry::updateSensor(const DynamicsBodies &bodies,
                                       double simTime) {
  if (std::remainder(simTime, 1.0 / freqHz_) <
      std::numeric_limits<double>::epsilon() * 100) {
    // Get the position of the robot this sensor is attached to
    auto position = bodies.getPosition(bodyIdx_);
    data_.emplace_back(position);
    timesteps_.emplace_back(simTime);
  }
}

} // namespace rb