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

// ---- PFG 3D record type tokens ----
// These must match the py_factor_graph Python reader/writer exactly.
inline constexpr const char *kVertexSe3Quat = "VERTEX_SE3:QUAT";
inline constexpr const char *kVertexSe3QuatPrior = "VERTEX_SE3:QUAT:PRIOR";
inline constexpr const char *kVertexXyz = "VERTEX_XYZ";
inline constexpr const char *kVertexXyzPrior = "VERTEX_XYZ:PRIOR";
inline constexpr const char *kEdgeSe3Quat = "EDGE_SE3:QUAT";
inline constexpr const char *kEdgeRange = "EDGE_RANGE";

/**
 * @name Covariance Serialization
 * @brief PFG covariance format: column-major lower-triangular unique elements.
 *
 * @details A symmetric NxN covariance matrix is serialized as N*(N+1)/2
 * space-separated values. The ordering is **column-major lower-triangular**:
 * iterate over columns left-to-right, and within each column emit rows from
 * the diagonal downward.
 *
 * This matches `py_factor_graph.utils.matrix_utils`:
 * - `get_list_column_major_from_symmetric_matrix()` (writer)
 * - `get_symmetric_matrix_from_list_column_major()` (reader)
 *
 * For a 3x3 matrix with indices (row, col):
 * @code
 *   c0 = (0,0)
 *   c1 = (1,0)   c3 = (1,1)
 *   c2 = (2,0)   c4 = (2,1)   c5 = (2,2)
 * @endcode
 * Emitted order: c0 c1 c2 c3 c4 c5 (6 elements)
 *
 * For a 6x6 matrix (pose covariance, tangent-space ordering
 * [x, y, z, roll, pitch, yaw]):
 * @code
 *   c0  = (0,0)
 *   c1  = (1,0)   c6  = (1,1)
 *   c2  = (2,0)   c7  = (2,1)   c11 = (2,2)
 *   c3  = (3,0)   c8  = (3,1)   c12 = (3,2)   c15 = (3,3)
 *   c4  = (4,0)   c9  = (4,1)   c13 = (4,2)   c16 = (4,3)   c18 = (4,4)
 *   c5  = (5,0)   c10 = (5,1)   c14 = (5,2)   c17 = (5,3)   c19 = (5,4)   c20 =
 * (5,5)
 * @endcode
 * Emitted order: c0 c1 c2 c3 c4 c5 c6 c7 ... c20 (21 elements)
 *
 * Diagonal indices for a diagonal-only 6x6: {0, 6, 11, 15, 18, 20}
 * Diagonal indices for a diagonal-only 3x3: {0, 3, 5}
 *
 * @note For diagonal matrices the column-major lower-triangular and row-major
 * upper-triangular orderings produce identical output because all off-diagonal
 * elements are zero.
 * @{
 */
inline constexpr int kCovUniqueElems6x6 = 21; ///< 6x6 symmetric: 6*7/2
inline constexpr int kCovUniqueElems3x3 = 6;  ///< 3x3 symmetric: 3*4/2
/** @} */

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

  /// Default pose prior covariance (6x6 symmetric, 21 unique elements).
  /// Serialized as column-major lower-triangular per PFG spec.
  /// Applied to first pose of each robot.
  std::array<double, kCovUniqueElems6x6> defaultPosePriorCov{};

  /// Per-landmark prior covariances (3x3 symmetric, 6 unique elements each).
  /// If empty, landmark priors are not written.
  /// If size < num landmarks, remaining landmarks use last entry.
  std::vector<std::array<double, kCovUniqueElems3x3>> landmarkPriorCovs{};

  /// Rotation variance for odometry edges.
  /// Translation variance is derived per-robot from PositionalXYOdometry noise.
  double odomRotationVariance{0.01};

  /// Rotation variance for GPS pose priors. Should be very large since GPS
  /// provides no rotation information. Translation covariance is derived
  /// per-robot from each GpsPosition sensor's noise model.
  double gpsRotationVariance{0.01};
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
 * @brief Fills a 6x6 diagonal covariance in PFG serialization order.
 * @details Places d0..d5 at the diagonal indices {0, 6, 11, 15, 18, 20}
 * of the 21-element column-major lower-triangular array. All off-diagonal
 * elements are zero. See @ref kCovUniqueElems6x6 for layout details.
 *
 * @param d0 Variance for x
 * @param d1 Variance for y
 * @param d2 Variance for z
 * @param d3 Variance for roll
 * @param d4 Variance for pitch
 * @param d5 Variance for yaw
 */
std::array<double, kCovUniqueElems6x6> makeDiagUpperTri6x6(double d0, double d1,
                                                           double d2, double d3,
                                                           double d4,
                                                           double d5);

/**
 * @brief Fills a 3x3 diagonal covariance in PFG serialization order.
 * @details Places d0..d2 at the diagonal indices {0, 3, 5} of the 6-element
 * column-major lower-triangular array. All off-diagonal elements are zero.
 * See @ref kCovUniqueElems3x3 for layout details.
 *
 * @param d0 Variance for x
 * @param d1 Variance for y
 * @param d2 Variance for z
 */
std::array<double, kCovUniqueElems3x3> makeDiagUpperTri3x3(double d0, double d1,
                                                           double d2);

} // namespace pyfg
