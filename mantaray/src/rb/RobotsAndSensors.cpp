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

PositionalOdomoetry::PositionalOdomoetry(
    double freqHz, int timeSteps, std::normal_distribution<double> noiseDist)
    : SensorI(timeSteps, freqHz), noiseDist_(noiseDist) {
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
                                       double simTime,
                                       std::mt19937 &rngEngine) {
  // TODO: Remove this frequency checking logic here
  if (std::fmod(simTime, 1.0 / freqHz_) <
      std::numeric_limits<double>::epsilon() * 100.0) {
    // Get the position of the robot this sensor is attached to
    auto position = bodies.getPosition(bodyIdx_);
    // TODO: Document this assumption about z noise not being included
    auto xNoise = noiseDist_(rngEngine);
    auto yNoise = noiseDist_(rngEngine);
    position.x() = position.x() * xNoise;
    position.y() = position.y() * yNoise;
    data_.emplace_back(position);
    timesteps_.emplace_back(simTime);
  }
}

} // namespace rb