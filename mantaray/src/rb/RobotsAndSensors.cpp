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

PositionalXYOdomoetry::PositionalXYOdomoetry(
    double freqHz, int timeSteps, std::normal_distribution<double> noiseDist)
    : SensorI(timeSteps, freqHz), noiseDist_(noiseDist) {
  if (getFreqHz() < 0) {
    throw std::invalid_argument("Frequency must be non-negative");
  }
}
std::vector<Eigen::VectorXd> PositionalXYOdomoetry::getSensorData() {
  return data_;
}

std::vector<double> PositionalXYOdomoetry::getSensorTimesteps() {
  return timesteps_;
}

void PositionalXYOdomoetry::updateSensor(const DynamicsBodies &bodies,
                                         double simTime,
                                         std::mt19937 &rngEngine) {
  // Get the position of the robot this sensor is attached to
  if (detail::isEqual(simTime, 0.0)) {
    prevPosition_ = bodies.getPosition(bodyIdx_);

    data_.emplace_back(prevPosition_);
    timesteps_.emplace_back(simTime);
    return;
  }
  auto velocity = bodies.getLinearVelocity(bodyIdx_);
  // TODO: Document this assumption about z noise not being included
  auto xNoise = noiseDist_(rngEngine);
  auto yNoise = noiseDist_(rngEngine);
  // Integrating but reusing vector
  velocity.x() = velocity.x() * dt_ + prevPosition_.x() + xNoise;
  velocity.y() = velocity.y() * dt_ + prevPosition_.y() + yNoise;
  velocity.z() = velocity.z() * dt_ + prevPosition_.z();
  data_.emplace_back(velocity);
  prevPosition_ = velocity;
  timesteps_.emplace_back(simTime);
}

} // namespace rb