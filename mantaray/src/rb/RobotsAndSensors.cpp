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
    : SensorI(timeSteps, SensorType::kPosOdomXY, freqHz),
      noiseDist_(noiseDist) {
  if (getFreqHz() < 0) {
    throw std::invalid_argument("Frequency must be non-negative");
  }
}
const std::vector<Eigen::VectorXd> &PositionalXYOdomoetry::getSensorData() {
  return data_;
}

const std::vector<double> &PositionalXYOdomoetry::getSensorTimesteps() {
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

GroundTruthPose::GroundTruthPose(double freqHz, int timeSteps)
    : SensorI(timeSteps, SensorType::kGroundTruthPose, freqHz) {}

void GroundTruthPose::updateSensor(const DynamicsBodies &bodies, double simTime,
                                   [[maybe_unused]] std::mt19937 &rngEngine) {
  data_.emplace_back(bodies.kinematics[bodyIdx_].poseGlobal.coeffs());
  timesteps_.emplace_back(simTime);
}
const std::vector<Eigen::VectorXd> &GroundTruthPose::getSensorData() {
  return data_;
}

const std::vector<double> &GroundTruthPose::getSensorTimesteps() {
  return timesteps_;
}

std::string commaSeperateVec(const Eigen::VectorXd data, size_t start,
                             size_t stop) {
  std::string output;
  constexpr char comma[] = ",";
  output.reserve((stop - start) + (stop - start));
  for (size_t i = start; i < stop - 1; i++) {
    output.append(std::to_string(data[i]));
    output.append(comma);
  }
  // Doing last step individually as we don't want a comma
  output.append(std::to_string(data[stop - 1]));
  return output;
}

const char *csvHeaderForSensor(const SensorType type) {
  switch (type) {
  case SensorType::kGroundTruthPose:
    return "x,y,z,q1,q2,q3,q4";
  case SensorType::kPosOdomXY:
    return "x,y,z";
  case SensorType::kGroundTruthTwist:
    return "vx,vy,vz,wx,wy,wz";
  case SensorType::kUnknown:
    throw std::invalid_argument("Unknown sensor type");
  default:
    const std::string msg =
        std::string("Unimplemented sensor type. Implement sensor type: ") +
        sensorTypeToString(type);
    throw std::invalid_argument(msg);
  }
}
std::string csvRowForSensor(const SensorType type, const Eigen::VectorXd data) {
  switch (type) {
  case SensorType::kGroundTruthPose:
    return commaSeperateVec(data, 0, data.size());
  case SensorType::kPosOdomXY:
    return commaSeperateVec(data, 0, data.size());
  case SensorType::kGroundTruthTwist:
    return commaSeperateVec(data, 0, data.size());
  case SensorType::kUnknown:
    throw std::invalid_argument("Unknown sensor type");
  default:
    const std::string msg =
        std::string("Unimplemented sensor type. Implement sensor type: ") +
        sensorTypeToString(type);
    throw std::invalid_argument(msg);
  }
}

void outputRobotSensorToCsv(const char *baseFilename, const RobotI &robot) {
  BodyIdx idx = robot.getBodyIdx();
  std::string currFilename = baseFilename;

  for (auto &sensor : robot.sensors_) {
    auto &data = sensor->getSensorData();
    auto &timestep = sensor->getSensorTimesteps();
    CHECK(data.size() == timestep.size(), "Mismatch in timestep and data size");

    currFilename.append(std::string("_") + std::to_string(idx) +
                        std::string("_") +
                        sensorTypeToString(sensor->sensorType_) + ".csv");

    // Open the file in write mode to overwrite
    std::ofstream csvFile(currFilename, std::ios::out);
    if (!csvFile.is_open()) {
      throw std::ios_base::failure("Failed to open file: " + currFilename);
    }

    // Write the header
    csvFile << "timestep," << csvHeaderForSensor(sensor->sensorType_) << "\n";

    // Write the data to the CSV file
    for (size_t i = 0; i < data.size(); ++i) {
      csvFile << timestep[i] << ","; // Write the timestep
      csvFile << csvRowForSensor(sensor->sensorType_, data[i])
              << "\n"; // Write the data
    }

    csvFile.close();             // Close the file
    currFilename = baseFilename; // Reset the filename for the next sensor
  }
}

} // namespace rb