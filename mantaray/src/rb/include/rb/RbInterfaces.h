/**
 * @file RbInterfaces.h
 * @brief Defines all interfaces that RbWorld expects to be fulfilled
 *
 * @details When adding new SensorType, ensure instructions are followed on
 * where values code needs to be added. A more procedural approach was taken
 * rather than inheritance based.
 */
#pragma once

#include <random>

#include "manif/manif.h"
#include "rb/PhysicsBodies.h"

namespace rb {
/**
 * @brief Enum to capture all the types of sensors
 * @details This can later be used to parse outputs easier based on these types
 *
 * When adding a new SensorType. Ensure you override the default kUnknown when
 * instantiating your sensor from SensorI. validateWorld does check this for
 * you. Additionally, if you want csv output or other outputs. Ensure to update
 * appropriate methods in RobotAndSensors.
 */
enum class SensorType {
  kUnknown,
  kGroundTruthPose,
  kGroundTruthTwist,
  kPosOdomXY,
};
/** @brief Provides a string conversion for the enum
 * @details No spaces so they make nice file names
 */
constexpr const char *sensorTypeToString(SensorType type) {
  switch (type) {
  case SensorType::kUnknown:
    return "Unknown";
  case SensorType::kGroundTruthPose:
    return "GroundTruthPose";
  case SensorType::kGroundTruthTwist:
    return "GroundTruthTwist";
  case SensorType::kPosOdomXY:
    return "PosOdomXY";
  default:
    return "InvalidSensor";
  }
}

/* @brief Defines the interface that RbWorld expects from sensors in order
 * to properly update them
 *
 * @par Sensor Update
 * Sensor update timing is managed by RbWorld and therefore requires the
 * inheritor to properly set frequency for SensorI. Additionally, sensor updates
 * are passed a global random number generator to avoid instantiating a unique
 * rng engine for each sensor.
 */
class SensorI {
public:
  /* @brief Constructs a sensor interface with preallocated vectors
   * @param freqHz: sensor update, set EXTREMELY high to fail if not overwritten
   */
  SensorI(int numTimesteps, SensorType type, double freqHz = 100000.0);
  virtual ~SensorI() =
      default; // Add virtual destructor to ensure proper cleanup

  BodyIdx bodyIdx_{0};
  SensorType sensorType_{SensorType::kUnknown};

  // Define pure virtual function for sensor data retrieval
  // Choosing to return references to avoid large data copies

  // Returns all sensor data time series
  virtual const std::vector<Eigen::VectorXd> &getSensorData() = 0;

  // Returns associated timesteps with sensor data
  virtual const std::vector<double> &getSensorTimesteps() = 0;

  /** @brief Pure virtual function to update sensors for inheritor to implement
   * @details provides rngEngine for the generation of noise if needed.
   * This function is only called at the correct dt! You do not need to check!
   */
  virtual void updateSensor(const DynamicsBodies &bodies, double simTime,
                            std::mt19937 &rngEngine) = 0;
  double getFreqHz() const;
  double getDt() const;

  void setDynamicsIndex(BodyIdx idx);
  bool checkUpdate() const;

protected:
  std::vector<double> timesteps_{};
  std::vector<Eigen::VectorXd> data_{};
  // Delta time at which the sensor updates, used to check sim dt
  const double dt_{0.0};
};

/** @brief Defines the interface that RbWorld expects from robots in order
 * to properly manage lifetimes, sensors, and compute twists.
 *
 * @property bodyIdx: index into kinematicData where robot's rigid body
 * data is stored
 *
 * @par Twist Update
 * The twist update is the key component of this interface. The integrator takes
 * twists from all robots and integrates them to create motion.
 *
 */
class RobotI {
public:
  virtual ~RobotI(); // Add virtual destructor to ensure proper cleanup

  void setDynamicsIndex(BodyIdx idx);
  BodyIdx getBodyIdx() const { return bodyIdx_; }

  virtual manif::SE3Tangentd
  computeLocalTwist(const DynamicsBodies &bodies) = 0; // Make pure virtual

  /**
   * @brief Adds a sensor
   * @return index: index to the sensor
   */
  size_t addSensor(std::unique_ptr<SensorI> sensor);

  // bodyIdx_ is provided by the RbWorld builder and gives access into dynamics
  // bodies
  BodyIdx bodyIdx_{0};

  // isAlive_ is a flag that determines if the robot is still within bounds
  bool isAlive_{true};
  std::vector<std::unique_ptr<SensorI>> sensors_;
};

} // namespace rb
