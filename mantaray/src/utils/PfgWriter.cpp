/**
 * @file PfgWriter.cpp
 * @brief Implementation of PFG (PyFactorGraphs) text file writer
 */

#include "mantaray/utils/PfgWriter.h"
#include "rb/PhysicsBodies.h"
#include "rb/RobotsAndSensors.h"

#include "fmt/format.h"
#include "spdlog/spdlog.h"

#include <algorithm>
#include <cassert>
#include <fstream>

namespace {

// ---------------------------------------------------------------------------
// Naming helpers
// ---------------------------------------------------------------------------

std::string robotName(size_t robotIdx, size_t timeIdx) {
  assert(robotIdx < 26 && "PFG format supports at most 26 robots (A-Z)");
  return fmt::format("{}{}", static_cast<char>('A' + robotIdx), timeIdx);
}

std::string landmarkName(size_t idx) { return fmt::format("L{}", idx); }

// ---------------------------------------------------------------------------
// Formatting helpers (PFG precision rules)
// ---------------------------------------------------------------------------

/// Prevents -0.0 from appearing in output
inline double noNegZero(double v) { return v == 0.0 ? 0.0 : v; }

/// Timestamp, translation, covariance: 9 decimal places
std::string fmtT(double v) { return fmt::format("{:.9f}", noNegZero(v)); }

/// Quaternion component: 7 decimal places
std::string fmtQ(double v) { return fmt::format("{:.7f}", noNegZero(v)); }

// ---------------------------------------------------------------------------
// Quaternion normalization
// ---------------------------------------------------------------------------

/// Ensures qw >= 0 by negating all four components if needed.
void normalizeQwPositive(double &qx, double &qy, double &qz, double &qw) {
  if (qw < 0.0) {
    qx = -qx;
    qy = -qy;
    qz = -qz;
    qw = -qw;
  }
}

// ---------------------------------------------------------------------------
// Sensor lookup
// ---------------------------------------------------------------------------

/// Returns pointer to the sensor of the given type, or nullptr.
rb::SensorI *findSensor(const rb::RobotI &robot, rb::SensorType type) {
  for (auto &sensor : robot.sensors_) {
    if (sensor->sensorType_ == type) {
      return sensor.get();
    }
  }
  return nullptr;
}

/// Returns the GroundTruthPose sensor, or nullptr.
rb::SensorI *findGtSensor(const rb::RobotI &robot) {
  return findSensor(robot, rb::SensorType::kGroundTruthPose);
}

/// Returns the PositionalXYOdometry sensor, or nullptr.
rb::PositionalXYOdometry *findOdomSensor(const rb::RobotI &robot) {
  auto *s = findSensor(robot, rb::SensorType::kPosOdomXY);
  return s ? static_cast<rb::PositionalXYOdometry *>(s) : nullptr;
}

// ---------------------------------------------------------------------------
// SE3 reconstruction from ground truth data vector
// ---------------------------------------------------------------------------

/// Builds manif::SE3d from a 7-element VectorXd [x,y,z,qx,qy,qz,qw].
manif::SE3d se3FromGtData(const Eigen::VectorXd &data) {
  Eigen::Quaterniond q(data[6], data[3], data[4], data[5]); // w,x,y,z ctor
  return manif::SE3d(Eigen::Vector3d(data[0], data[1], data[2]), q);
}

// ---------------------------------------------------------------------------
// Nearest timestamp index lookup
// ---------------------------------------------------------------------------

/// Finds the index in `timestamps` closest to `queryTime`.
size_t findNearestTimeIndex(const std::vector<double> &timestamps,
                            double queryTime) {
  assert(!timestamps.empty());
  auto it = std::lower_bound(timestamps.begin(), timestamps.end(), queryTime);

  if (it == timestamps.end()) {
    return timestamps.size() - 1;
  }
  if (it == timestamps.begin()) {
    return 0;
  }

  size_t idx = static_cast<size_t>(std::distance(timestamps.begin(), it));
  // Pick whichever of idx and idx-1 is closer
  double distPrev = queryTime - timestamps[idx - 1];
  double distCurr = timestamps[idx] - queryTime;
  return (distPrev <= distCurr) ? idx - 1 : idx;
}

// ---------------------------------------------------------------------------
// Covariance formatting
// ---------------------------------------------------------------------------

template <size_t N>
std::string formatUpperTri(const std::array<double, N> &cov) {
  std::string result;
  for (size_t i = 0; i < N; ++i) {
    if (i > 0)
      result += ' ';
    result += fmtT(cov[i]);
  }
  return result;
}

// ---------------------------------------------------------------------------
// Pose writing helpers
// ---------------------------------------------------------------------------

/// Writes qx qy qz qw from a 7-element GT data vector, with qw>=0
/// normalization.
std::string formatPoseQuaternion(const Eigen::VectorXd &data) {
  double qx = data[3], qy = data[4], qz = data[5], qw = data[6];
  normalizeQwPositive(qx, qy, qz, qw);
  return fmt::format("{} {} {} {}", fmtQ(qx), fmtQ(qy), fmtQ(qz), fmtQ(qw));
}

/// Writes x y z from a data vector.
std::string formatTranslation(const Eigen::VectorXd &data) {
  return fmt::format("{} {} {}", fmtT(data[0]), fmtT(data[1]), fmtT(data[2]));
}

/// Writes x y z qx qy qz qw from an SE3d pose.
std::string formatSE3(const manif::SE3d &pose) {
  auto t = pose.translation();
  auto q = pose.quat();
  double qx = q.x(), qy = q.y(), qz = q.z(), qw = q.w();
  normalizeQwPositive(qx, qy, qz, qw);
  return fmt::format("{} {} {} {} {} {} {}", fmtT(t.x()), fmtT(t.y()),
                     fmtT(t.z()), fmtQ(qx), fmtQ(qy), fmtQ(qz), fmtQ(qw));
}

// ---------------------------------------------------------------------------
// Endpoint → PFG name
// ---------------------------------------------------------------------------

std::string
endpointName(const sim::RangeEndpoint &ep, double queryTime,
             const std::vector<std::vector<double>> &robotGtTimestamps) {
  switch (ep.type) {
  case sim::EndpointType::kLandmark:
    return landmarkName(ep.index);
  case sim::EndpointType::kRobot: {
    size_t timeIdx =
        findNearestTimeIndex(robotGtTimestamps[ep.index], queryTime);
    return robotName(ep.index, timeIdx);
  }
  }
  // Unreachable, but satisfy compiler
  return {};
}

} // anonymous namespace

// ===========================================================================
// Public API
// ===========================================================================

namespace pyfg {

std::array<double, 21> makeDiagUpperTri6x6(double d0, double d1, double d2,
                                           double d3, double d4, double d5) {
  std::array<double, 21> out{};
  // Upper triangle row-major: (0,0) (0,1) (0,2) (0,3) (0,4) (0,5)
  //                                 (1,1) (1,2) (1,3) (1,4) (1,5) ...
  // Diagonal elements are at positions: 0, 6, 11, 15, 18, 20
  out[0] = d0;
  out[6] = d1;
  out[11] = d2;
  out[15] = d3;
  out[18] = d4;
  out[20] = d5;
  return out;
}

std::array<double, 6> makeDiagUpperTri3x3(double d0, double d1, double d2) {
  std::array<double, 6> out{};
  // (0,0) (0,1) (0,2) (1,1) (1,2) (2,2)
  out[0] = d0;
  out[3] = d1;
  out[5] = d2;
  return out;
}

void writePfg(const std::string &filename, const rb::RbWorld &world,
              const std::vector<sim::RangeMeasurement> &measurements,
              const PfgWriterConfig &config) {

  std::ofstream file(filename);
  if (!file.is_open()) {
    SPDLOG_ERROR("Failed to open PFG output file: {}", filename);
    throw std::runtime_error("Cannot open PFG file: " + filename);
  }

  const size_t numRobots = world.robots.size();
  const size_t numLandmarks = world.landmarks.size();

  // TODO: GPS Measurements need to be included as priors
  /* TODO: Need to ensure rotations sigma is very low to prevent rotations
   * from influencing solver
   */
  // TODO: Double check mapping between factors, variables, and edges

  // =========================================================================
  // Section 1: VERTEX_SE3:QUAT — pose variables
  // =========================================================================
  for (size_t i = 0; i < numRobots; ++i) {
    auto *gt = findGtSensor(*world.robots[i]);
    if (!gt) {
      SPDLOG_WARN("Robot {} has no GroundTruthPose sensor, skipping vertices",
                  i);
      continue;
    }
    const auto &data = gt->getSensorData();
    const auto &timestamps = gt->getSensorTimesteps();
    for (size_t t = 0; t < data.size(); ++t) {
      file << "VERTEX_SE3:QUAT " << fmtT(timestamps[t]) << ' '
           << robotName(i, t) << ' ' << formatTranslation(data[t]) << ' '
           << formatPoseQuaternion(data[t]) << '\n';
    }
  }

  // =========================================================================
  // Section 2: VERTEX_XYZ — landmark variables
  // =========================================================================
  for (size_t j = 0; j < numLandmarks; ++j) {
    const auto &pos = world.landmarks[j];
    file << "VERTEX_XYZ " << landmarkName(j) << ' ' << fmtT(pos.x()) << ' '
         << fmtT(pos.y()) << ' ' << fmtT(pos.z()) << '\n';
  }

  // =========================================================================
  // Section 3: VERTEX_SE3:QUAT:PRIOR — pose priors (first pose per robot)
  // =========================================================================
  for (size_t i = 0; i < numRobots; ++i) {
    auto *gt = findGtSensor(*world.robots[i]);
    if (!gt)
      continue;
    const auto &data = gt->getSensorData();
    const auto &timestamps = gt->getSensorTimesteps();
    if (data.empty())
      continue;

    file << "VERTEX_SE3:QUAT:PRIOR " << fmtT(timestamps[0]) << ' '
         << robotName(i, 0) << ' ' << formatTranslation(data[0]) << ' '
         << formatPoseQuaternion(data[0]) << ' '
         << formatUpperTri(config.defaultPosePriorCov) << '\n';
  }

  // =========================================================================
  // Section 4: VERTEX_XYZ:PRIOR — landmark priors
  // =========================================================================
  if (!config.landmarkPriorCovs.empty()) {
    for (size_t j = 0; j < numLandmarks; ++j) {
      // Use per-landmark cov if available, otherwise last entry
      size_t covIdx = std::min(j, config.landmarkPriorCovs.size() - 1);
      const auto &pos = world.landmarks[j];
      file << "VERTEX_XYZ:PRIOR " << fmtT(0.0) << ' ' << landmarkName(j) << ' '
           << fmtT(pos.x()) << ' ' << fmtT(pos.y()) << ' ' << fmtT(pos.z())
           << ' ' << formatUpperTri(config.landmarkPriorCovs[covIdx]) << '\n';
    }
  }

  // =========================================================================
  // Section 5: EDGE_SE3:QUAT — odometry edges
  // =========================================================================
  for (size_t i = 0; i < numRobots; ++i) {
    // Build per-robot odom covariance from sensor noise
    auto *odomSensor = findOdomSensor(*world.robots[i]);
    std::array<double, 21> odomCov{};
    if (odomSensor) {
      double var = odomSensor->getNoiseStddev() * odomSensor->getNoiseStddev();
      odomCov = makeDiagUpperTri6x6(var, var, var, config.odomRotationVariance,
                                    config.odomRotationVariance,
                                    config.odomRotationVariance);
    } else {
      SPDLOG_WARN("Robot {} has no odom sensor, using rotation variance only",
                  i);
      odomCov = makeDiagUpperTri6x6(0.0, 0.0, 0.0, config.odomRotationVariance,
                                    config.odomRotationVariance,
                                    config.odomRotationVariance);
    }

    if (config.useGroundTruthOdometry) {
      // Use ground truth poses for noiseless relative transforms
      auto *gt = findGtSensor(*world.robots[i]);
      if (!gt || gt->getSensorData().size() < 2)
        continue;
      const auto &data = gt->getSensorData();
      const auto &timestamps = gt->getSensorTimesteps();

      for (size_t t = 0; t < data.size() - 1; ++t) {
        manif::SE3d poseFrom = se3FromGtData(data[t]);
        manif::SE3d poseTo = se3FromGtData(data[t + 1]);
        manif::SE3d rel;
        rb::relativeTransformBodyFrame(poseFrom, poseTo, rel);

        file << "EDGE_SE3:QUAT " << fmtT(timestamps[t + 1]) << ' '
             << robotName(i, t) << ' ' << robotName(i, t + 1) << ' '
             << formatSE3(rel) << ' ' << formatUpperTri(odomCov) << '\n';
      }
    } else {
      // Use odom sensor positions (noisy), identity rotation
      if (!odomSensor || odomSensor->getSensorData().size() < 2)
        continue;
      const auto &data = odomSensor->getSensorData();
      const auto &timestamps = odomSensor->getSensorTimesteps();

      for (size_t t = 0; t < data.size() - 1; ++t) {
        Eigen::Vector3d delta = data[t + 1].head<3>() - data[t].head<3>();

        file << "EDGE_SE3:QUAT " << fmtT(timestamps[t + 1]) << ' '
             << robotName(i, t) << ' ' << robotName(i, t + 1) << ' '
             << fmtT(delta.x()) << ' ' << fmtT(delta.y()) << ' '
             << fmtT(delta.z()) << ' ' << fmtQ(0.0) << ' ' << fmtQ(0.0) << ' '
             << fmtQ(0.0) << ' ' << fmtQ(1.0) << ' ' << formatUpperTri(odomCov)
             << '\n';
      }
    }
  }

  // =========================================================================
  // Section 8: EDGE_RANGE — range measurements
  // =========================================================================

  // Pre-build per-robot GT timestamp lookup
  std::vector<std::vector<double>> robotGtTimestamps(numRobots);
  for (size_t i = 0; i < numRobots; ++i) {
    auto *gt = findGtSensor(*world.robots[i]);
    if (gt) {
      robotGtTimestamps[i] = gt->getSensorTimesteps();
    }
  }

  for (const auto &meas : measurements) {
    if (meas.status != sim::RangeStatus::kOk)
      continue;

    std::string pingerName =
        endpointName(meas.pinger, meas.simTimeSec, robotGtTimestamps);
    std::string targetName =
        endpointName(meas.target, meas.simTimeSec, robotGtTimestamps);

    file << "EDGE_RANGE " << fmtT(meas.simTimeSec) << ' ' << pingerName << ' '
         << targetName << ' ' << fmtT(static_cast<double>(meas.rangeMeters))
         << ' ' << fmtT(config.rangeVariance) << '\n';
  }

  file.close();
  SPDLOG_INFO("PFG file written: {} ({} robots, {} landmarks)", filename,
              numRobots, numLandmarks);
}

} // namespace pyfg
