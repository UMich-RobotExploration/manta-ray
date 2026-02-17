//
// Created by tko on 2/11/26.
//

#include "rb/pch.h"

#include "rb/RbInterfaces.h"
#include "rb/RbWorld.h"

namespace rb {
void RbWorld::reserveRobots(size_t count) {
  robots.reserve(count);
  dynamicsBodies.reserveBodies(count);
}
void RbWorld::reserveLandmarks(size_t count) { landmarks.reserve(count); }
void RbWorld::addLandmark(const Eigen::Vector3d &landmark) {
  landmarks.emplace_back(landmark);
}

/*
 * @brief Gathers the local twists from each robot and stores them in the bodies
 * @details Sets all dead robots' twist to zeros. Prevents conditionals in
 * update loop
 */
void gatherTwists(RbWorld &world) {
  for (auto &robot : world.robots) {
    if (robot->isAlive_) {
      auto twist = robot->computeLocalTwist(world.dynamicsBodies);
      world.dynamicsBodies.getKinematicData(robot->bodyIdx_).twistLocal = twist;
    } else {
      world.dynamicsBodies.getKinematicData(robot->bodyIdx_)
          .twistLocal.setZero();
    }
  }
}

void updateSensors(RbWorld &world) {
  for (auto &robot : world.robots) {
    if (robot->isAlive_) {
      for (auto &sensor : robot->sensors_) {
        sensor->updateSensor(world.dynamicsBodies, world.simData.time);
      }
    }
  }
}

void RbWorld::validateWorld() {
  const double simDataHz = 1.0 / simData.dt;
  for (const auto &robot : robots) {
    if (robot->bodyIdx_ >= dynamicsBodies.kinematics.size()) {
      std::string msg = "Robot with body index " +
                        std::to_string(robot->bodyIdx_) +
                        " has an invalid index that exceeds the number of "
                        "bodies in the world.";
      throw std::runtime_error(msg);
    }
    for (const auto &sensor : robot->sensors_) {
      if (!sensor->checkUpdate()) {
        std::string msg = "Sensor attached to robot with body index " +
                          std::to_string(robot->bodyIdx_) +
                          " has a mismatch between data and timesteps size.";
        throw std::runtime_error(msg);
      }
      double sensorFreqHz = sensor->getFreqHz();
      if (sensorFreqHz > simDataHz) {
        std::string msg =
            "Sensor attached to robot with body index " +
            std::to_string(robot->bodyIdx_) +
            " has an update frequency that is too high for the "
            "simulation timestep." +
            " Sim Frequency is: " + std::to_string(simDataHz) +
            ", Sensor Frequency is: " + std::to_string(sensorFreqHz) +
            " This likely means you did not provide a value upon construction "
            "to the inteface class or you need to change the simulation dt.";
        throw std::invalid_argument(msg);
      }
      if (std::remainder(simDataHz, sensorFreqHz) >
          std::numeric_limits<double>::epsilon() * 100) {
        std::string msg =
            "Sensor attached to robot with body index " +
            std::to_string(robot->bodyIdx_) +
            " has an update frequency that is not an integer multiple of the "
            "simulation timestep." +
            " Sim Frequency is: " + std::to_string(simDataHz) +
            ", Sensor Frequency is: " + std::to_string(sensorFreqHz) +
            " This may lead to missed updates or updates that are not aligned "
            "with the sim time. Consider adjusting the sensor frequency or "
            "simulation dt to be integer multiples.";
        throw std::invalid_argument(msg);
      }
    }
  }
}

/*
 * @brief Advances the world dt seconds
 * @details *UPDATES* Sim time. It is the sole updated of sim time.
 */
void RbWorld::stepWorld(double dt) {
  gatherTwists(*this);
  for (auto &kinematicData : dynamicsBodies.kinematics) {
    manif::SE3d newPose;
    integratePose(kinematicData.poseGlobal, kinematicData.twistLocal, dt,
                  newPose);
    kinematicData.poseGlobal = newPose;
  }
  simData.time += dt;
}

/**
 * @brief Advances the world to the specified absolute time
 * @param time desired time to advance to (absolute)
 * @throws runtime_error if request is backwards in time
 */
void RbWorld::advanceWorld(double time) {
  if (detail::isEqual(simData.time, 0.0)) {
    validateWorld();
  }
  if (detail::isEqual(time, simData.time)) {
    return; // No advancement needed
  } else if (time < simData.time) {
    std::string msg = "Cannot simulate the requested step as it is backwards "
                      "in time. Current time: " +
                      std::to_string(simData.time) +
                      ", requested time: " + std::to_string(time);
    throw std::runtime_error(msg);
  }
  if ((simData.time > 0) && (std::remainder(simData.time, simData.dt) >
                             std::numeric_limits<double>::epsilon() * 100)) {
    // TODO: We need to make sure that we get back to standard dt timesteps
    // if start time or time input is not an integer multiple of dt.
    std::string msg = "Current simulation does not allow non dt aligned "
                      "timesteps. It is a TODO";
    throw std::invalid_argument(msg);
  } else {
    double timeToAdvance = time - simData.time;
    while (timeToAdvance > 0) {
      double dt = std::min(simData.dt, timeToAdvance);
      stepWorld(dt);
      updateSensors(*this);
      timeToAdvance -= dt;
    }
    return;
  }
}

namespace detail {
bool isEqual(double x, double y) {
  return std::abs(x - y) < kBoundaryEpsilonDouble;
}
} // namespace detail

} // namespace rb
