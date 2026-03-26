/** @file PfgWriter.h
 * @brief Writer for the PyFactorGraphs (.pfg) text format
 */
#pragma once

#include "mantaray/sim/AcousticPairwiseRangeSystem.h"
#include "rb/RbWorld.h"
#include <array>
#include <string>
#include <vector>

/** @namespace pyfg
 * @brief PyFactorGraphs file format utilities
 */
namespace pyfg {

/**
 * @brief Configuration for the PFG writer.
 *
 * @details Controls which data source is used for odometry edges, and provides
 * covariance parameters for priors and measurements. Per-robot odometry
 * covariance is derived automatically from each robot's PositionalXYOdometry
 * sensor noise model.
 */
struct PfgWriterConfig {
  /// If true, odometry edges use GroundTruthPose (noiseless).
  /// If false, odometry edges use PositionalXYOdometry sensor data.
  bool useGroundTruthOdometry{true};

  /// Range measurement variance (scalar, for EDGE_RANGE)
  double rangeVariance{1.0};

  /// Default pose prior covariance (6x6 upper-tri, 21 elements).
  /// Applied to first pose of each robot.
  std::array<double, 21> defaultPosePriorCov{};

  /// Per-landmark prior covariances (3x3 upper-tri, 6 elements each).
  /// If empty, landmark priors are not written.
  /// If size < num landmarks, remaining landmarks use last entry.
  std::vector<std::array<double, 6>> landmarkPriorCovs{};

  /// Rotation variance for odometry edges.
  /// Translation variance is derived per-robot from PositionalXYOdometry noise.
  double odomRotationVariance{0.04};
};

/**
 * @brief Writes a PFG factor graph file from simulation data.
 *
 * @details Iterates over all robots and their sensors to produce a complete
 * factor graph in PFG text format. Sections are written in the order required
 * by the PFG spec: pose vertices, landmark vertices, pose priors, landmark
 * priors, odometry edges, then range edges.
 *
 * @param filename Output file path (e.g., "output.pfg")
 * @param world The simulation world (robots with sensors, landmarks)
 * @param measurements Range measurements from AcousticPairwiseRangeSystem
 * @param config Writer configuration (covariance params, GT vs odom flag)
 */
void writePfg(const std::string &filename, const rb::RbWorld &world,
              const std::vector<sim::RangeMeasurement> &measurements,
              const PfgWriterConfig &config);

/**
 * @brief Fills a 6x6 upper-triangle array from 6 diagonal variances.
 * @details Off-diagonal elements are zero.
 * Element order: (0,0) (0,1) ... (0,5) (1,1) (1,2) ... (5,5) = 21 elements.
 */
std::array<double, 21> makeDiagUpperTri6x6(double d0, double d1, double d2,
                                           double d3, double d4, double d5);

/**
 * @brief Fills a 3x3 upper-triangle array from 3 diagonal variances.
 * @details Off-diagonal elements are zero.
 * Element order: (0,0) (0,1) (0,2) (1,1) (1,2) (2,2) = 6 elements.
 */
std::array<double, 6> makeDiagUpperTri3x3(double d0, double d1, double d2);

} // namespace pyfg
