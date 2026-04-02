/** @file SimConfig.h
 * @brief Top-level simulation configuration and JSON deserialization
 */
#pragma once

#include <Eigen/Core>
#include <fstream>
#include <json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "acoustics/SimulationConfig.h"
#include "mantaray/sim/AcousticPairwiseRangeSystem.h"
#include "mantaray/sim/CurrentDriftRobot.h"
#include "mantaray/sim/RobotFactory.h"
#include "mantaray/utils/PfgWriter.h"
#include "rb/RobotsAndSensors.h"

// Eigen::Vector3d serializer — must be in nlohmann namespace
namespace nlohmann {
template <> struct adl_serializer<Eigen::Vector3d> {
  static void from_json(const json &j, Eigen::Vector3d &v) {
    auto a = j.get<std::array<double, 3>>();
    v = Eigen::Vector3d(a[0], a[1], a[2]);
  }
};
} // namespace nlohmann

// from_json overloads — each in the namespace of its target type for ADL

namespace rb {
inline void from_json(const nlohmann::json &j, ConstantVelConfig &c) {
  j.at("position").get_to(c.position);
  j.at("velocity").get_to(c.velocity);
}
} // namespace rb

namespace robots {
inline void from_json(const nlohmann::json &j, CurrentDriftConfig &c) {
  j.at("position").get_to(c.position);
  c.targetDepth = j.value("target_depth", c.targetDepth);
  c.holdSeconds = j.value("hold_seconds", c.holdSeconds);
  c.surfaceHoldSeconds = j.value("surface_hold_seconds", c.surfaceHoldSeconds);
  c.verticalSpeed = j.value("vertical_speed", c.verticalSpeed);
  c.surfaceDepth = j.value("surface_depth", c.surfaceDepth);
}
} // namespace robots

namespace sim {
inline void from_json(const nlohmann::json &j, StandardSensorConfig &c) {
  c.gtFreqHz = j.value("gt_freq_hz", c.gtFreqHz);
  c.odomFreqHz = j.value("odom_freq_hz", c.odomFreqHz);
  c.gpsFreqHz = j.value("gps_freq_hz", c.gpsFreqHz);
  c.odomNoiseStddev = j.value("odom_noise_stddev", c.odomNoiseStddev);
  c.gpsXyNoiseStddev = j.value("gps_xy_noise_stddev", c.gpsXyNoiseStddev);
  c.gpsZNoiseStddev = j.value("gps_z_noise_stddev", c.gpsZNoiseStddev);
}
} // namespace sim

namespace pyfg {
inline void from_json(const nlohmann::json &j, PfgWriterConfig &c) {
  c.useGroundTruthOdometry =
      j.value("use_ground_truth_odometry", c.useGroundTruthOdometry);
  c.rangeVariance = j.value("range_variance", c.rangeVariance);
  c.odomRotationVariance =
      j.value("odom_rotation_variance", c.odomRotationVariance);
  c.gpsRotationVariance =
      j.value("gps_rotation_variance", c.gpsRotationVariance);

  if (j.contains("pose_prior_cov_diag")) {
    auto d = j.at("pose_prior_cov_diag").get<std::array<double, 6>>();
    c.defaultPosePriorCov =
        makeDiagUpperTri6x6(d[0], d[1], d[2], d[3], d[4], d[5]);
  }
  if (j.contains("landmark_prior_cov_diag")) {
    auto d = j.at("landmark_prior_cov_diag").get<std::array<double, 3>>();
    c.landmarkPriorCovs.push_back(makeDiagUpperTri3x3(d[0], d[1], d[2]));
  }
}
} // namespace pyfg

namespace config {

using json = nlohmann::json;

enum class RobotType { kConstantVel, kCurrentDrift };

inline RobotType robotTypeFromString(const std::string &s) {
  if (s == "constant_vel")
    return RobotType::kConstantVel;
  if (s == "current_drift")
    return RobotType::kCurrentDrift;
  throw std::runtime_error("Unknown robot type: " + s);
}

struct SimConfig {
  std::string runName{"sim"};
  std::string outputDir;
  std::string envConfigFile;
  size_t rngSeed{10020};
  size_t bellhopMemoryMib{80};

  double endTimeHours{8.0};
  double physicsDt{0.1};
  double pingIntervalMin{30.0};
  double boundsCheckIntervalSec{100.0};

  std::string tofMode{"one_way"};
  double debugRangeErrorPct{0.0};

  sim::StandardSensorConfig sensors{};

  json robotsJson;
  std::vector<Eigen::Vector3d> landmarks;

  pyfg::PfgWriterConfig pfg{};
};

inline void from_json(const json &j, SimConfig &c) {
  c.runName = j.value("run_name", c.runName);
  j.at("output_dir").get_to(c.outputDir);
  j.at("env_config_file").get_to(c.envConfigFile);
  c.rngSeed = j.value("rng_seed", c.rngSeed);
  c.bellhopMemoryMib = j.value("bellhop_memory_mib", c.bellhopMemoryMib);

  if (j.contains("timing")) {
    const auto &t = j.at("timing");
    c.endTimeHours = t.value("end_time_hours", c.endTimeHours);
    c.physicsDt = t.value("physics_dt", c.physicsDt);
    c.pingIntervalMin = t.value("ping_interval_min", c.pingIntervalMin);
    c.boundsCheckIntervalSec =
        t.value("bounds_check_interval_sec", c.boundsCheckIntervalSec);
  }

  if (j.contains("acoustics")) {
    const auto &a = j.at("acoustics");
    c.tofMode = a.value("tof_mode", c.tofMode);
    c.debugRangeErrorPct =
        a.value("debug_range_error_pct", c.debugRangeErrorPct);
  }

  if (j.contains("sensors")) {
    j.at("sensors").get_to(c.sensors);
  }

  if (j.contains("robots")) {
    c.robotsJson = j.at("robots");
  }

  if (j.contains("landmarks")) {
    c.landmarks = j.at("landmarks").get<std::vector<Eigen::Vector3d>>();
  }

  if (j.contains("pfg")) {
    j.at("pfg").get_to(c.pfg);
  }
}

inline SimConfig loadSimConfig(const std::string &path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error("Cannot open sim config file: " + path);
  }
  auto j = json::parse(f);
  return j.get<SimConfig>();
}

} // namespace config
