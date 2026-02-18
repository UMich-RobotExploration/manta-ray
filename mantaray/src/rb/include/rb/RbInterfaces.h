//
// Created by tko on 2/11/26.
//

#pragma once

#include <random>

#include "manif/manif.h"
#include "rb/PhysicsBodies.h"

namespace rb {
class SensorI {
public:
  /* @brief Constructs a sensor interface with preallocated vectors
   * @param freqHz: sensor update, set EXTREMELY high to fail if not overwritten
   */
  SensorI(int numTimesteps, double freqHz = 100000.0);
  virtual ~SensorI() =
      default; // Add virtual destructor to ensure proper cleanup

  BodyIdx bodyIdx_{0};

  // Define pure virtual function for sensor data retrieval
  virtual std::vector<Eigen::VectorXd> getSensorData() = 0;
  virtual std::vector<double> getSensorTimesteps() = 0;

  // @brief Pure virtual function to update sensors for inheritor to implement
  // @details provides rngEngine for the generation of noise if needed.
  // This function is only called at the correct dt! You do not need to check!
  virtual void updateSensor(const DynamicsBodies &bodies, double simTime,
                            std::mt19937 &rngEngine) = 0;
  double getFreqHz() const;
  double getDt() const;

  void setDynamicsIndex(BodyIdx idx);
  bool checkUpdate();

protected:
  std::vector<double> timesteps_{};
  std::vector<Eigen::VectorXd> data_{};
  // Frequency at which the sensor updates, used to check sim dt
  const double dt_{0.0};
};

class RobotI {
public:
  virtual ~RobotI(); // Add virtual destructor to ensure proper cleanup

  void setDynamicsIndex(BodyIdx idx);
  BodyIdx getBodyIdx() const { return bodyIdx_; }

  virtual manif::SE3Tangentd
  computeLocalTwist(const DynamicsBodies &bodies) = 0; // Make pure virtual
  void addSensor(std::unique_ptr<SensorI> sensor);

  // bodyIdx_ is provided by the RbWorld builder and gives access into dynamics
  // bodies
  BodyIdx bodyIdx_{0};
  // isAlive_ is a flag that determines if the robot is still within bounds
  bool isAlive_{true};
  std::vector<std::unique_ptr<SensorI>> sensors_;
};

} // namespace rb
