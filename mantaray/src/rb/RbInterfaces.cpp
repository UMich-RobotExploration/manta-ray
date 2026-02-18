
#include "rb/RbInterfaces.h"

namespace rb {
RobotI::~RobotI() = default; // Add destructor definition

void RobotI::setDynamicsIndex(BodyIdx idx) { bodyIdx_ = idx; }
size_t RobotI::addSensor(std::unique_ptr<SensorI> sensor) {
  size_t newIdx = sensors_.size();
  sensors_.emplace_back(std::move(sensor));
  sensors_.back()->setDynamicsIndex(
      bodyIdx_); // Set the sensor's body index to match the robot's
  return newIdx;
}

SensorI::SensorI(int numTimesteps, SensorType type, double freqHz)
    : sensorType_(type), dt_(1.0 / freqHz) {
  timesteps_.reserve(numTimesteps);
  data_.reserve(numTimesteps);
}
void SensorI::setDynamicsIndex(BodyIdx idx) { bodyIdx_ = idx; }
bool SensorI::checkUpdate() { return (data_.size() == timesteps_.size()); }
double SensorI::getFreqHz() const { return 1.0 / dt_; }
double SensorI::getDt() const { return dt_; }
} // namespace rb