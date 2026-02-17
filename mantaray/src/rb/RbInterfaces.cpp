
#include "rb/RbInterfaces.h"

namespace rb {
RobotI::~RobotI() = default; // Add destructor definition

void RobotI::setDynamicsIndex(BodyIdx idx) { bodyIdx_ = idx; }
void RobotI::addSensor(std::unique_ptr<SensorI> sensor) {
  sensors_.emplace_back(std::move(sensor));
  sensors_.back()->setDynamicsIndex(
      bodyIdx_); // Set the sensor's body index to match the robot's
}

SensorI::SensorI(int numTimesteps, double freqHz) : freqHz_(freqHz) {
  timesteps_.reserve(numTimesteps);
  data_.reserve(numTimesteps);
}
void SensorI::setDynamicsIndex(BodyIdx idx) { bodyIdx_ = idx; }
bool SensorI::checkUpdate() { return (data_.size() == timesteps_.size()); }
double SensorI::getFreqHz() const { return freqHz_; }
} // namespace rb