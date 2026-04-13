/** @file AcousticPairwiseRangeSystem.h
 * @brief Pairwise acoustic range measurement system using Bellhop ray tracing
 */

#pragma once

#include "acoustics/AcousticsBuilder.h"
#include "acoustics/Arrival.h"
#include "acoustics/BellhopContext.h"
#include "acoustics/helpers.h"
#include "mantaray/utils/Logger.h"
#include "rb/RbWorld.h"

#include "fmt/format.h"
#include <cmath>
#include <cstring>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

/** @namespace sim
 * @brief Simulation systems, acoustic ranging, and robot factory utilities
 */
namespace sim {

/** @brief Selects one-way or two-way time-of-flight scaling. */
enum class GlobalTofMode { kOneWay, kTwoWay };

/**
 * @brief Outcome of a single range measurement attempt.
 */
enum class RangeStatus {
  /// Successful measurement
  kOk,
  /// Pinger was already marked dead
  kSkippedPingerDead,
  /// Target was already marked dead
  kSkippedTargetDead,
  /// One or both endpoints outside the acoustic domain
  kOutOfBounds,
  /// Bellhop produced no valid arrival
  kNoArrival,
  /// Sound speed profile query returned invalid value
  kSspSampleFailed,
};

/** @brief Sentinel value for invalid or unavailable distance/speed/TOF fields.
 */
constexpr float kInvalidDistance = -1.0f;

/** @brief Distinguishes robots from static landmarks. */
enum class EndpointType { kRobot, kLandmark };

/**
 * @brief Identifies one end of an acoustic link.
 */
struct RangeEndpoint {
  /// Whether this is a robot or landmark
  EndpointType type{EndpointType::kRobot};
  /// Index into the world's robot or landmark list
  size_t index{0};
};

/**
 * @brief Self-contained record of a single acoustic range measurement.
 *
 * @details Stores everything needed for post-processing, plotting, and factor
 * graph construction (gtsam / PyFactorGraphs) without requiring access to the
 * originating link or simulation state.
 */
struct RangeMeasurement {
  /// Simulation time when the measurement was taken
  double simTimeSec{0.0};
  /// Endpoint that emitted the acoustic ping
  RangeEndpoint pinger{};
  /// Endpoint that received the acoustic ping
  RangeEndpoint target{};
  /// Outcome of the measurement attempt
  RangeStatus status{RangeStatus::kNoArrival};
  /// Computed range in meters (kInvalidDistance if invalid)
  float rangeMeters{kInvalidDistance};
  /// Effective time-of-flight after mode scaling
  float tofEffectiveSec{kInvalidDistance};
  /// Sound speed sampled at the pinger position
  float soundSpeedAtPingerMps{kInvalidDistance};
};

/// @brief Diagnostic output from the iterative beam solver.
struct TofConvergenceInfo {
  /// Bellhop runs executed (1 = no retry)
  int iterations{1};
  /// Beam count at resolution (or last tried)
  int finalBeams{0};
  /// True if TOF converged within tolerance
  bool converged{false};
  /// True if result came from reciprocal cache
  bool fromCache{false};
  /// True if accepted TOF was from multipath
  bool multipathUsed{false};
  /// |TOF_curr - TOF_prev| at final comparison
  float lastDelta{0.0f};
};

/**
 * @brief Directed link between two endpoints.
 *
 * @details Used internally to track which pairs to run Bellhop on.
 */
struct RangeLink {
  /// Source of the acoustic ping
  RangeEndpoint pinger{};
  /// Receiver of the acoustic ping
  RangeEndpoint target{};
};

/**
 * @brief Manages pairwise acoustic ranging between all robots and landmarks.
 *
 * @details Builds directed links for every robot-to-robot and robot-to-landmark
 * pair, runs Bellhop ray tracing to obtain time-of-flight, and converts TOF to
 * range using the local sound speed profile. Robot-robot pairs exploit acoustic
 * reciprocity to halve the number of Bellhop runs.
 *
 * Measurements are accumulated in a flat chronological log accessible via
 * getMeasurements(). A lightweight checkBounds() method can be called at
 * physics-rate to cull out-of-bounds robots without running Bellhop.
 *
 * @section iterative_beam_solver Iterative Beam Refinement
 *
 * When acquiring time-of-flight, the system extracts both direct-path (zero
 * bounce) and any-path (fastest regardless of bounces) arrivals from each
 * Bellhop run via a single pass (ArrivalPair).
 *
 * **Direct path**: Accepted immediately on first detection. The direct path
 * is a unique geometric path whose TOF is accurate when found, so no
 * convergence verification is required.
 *
 * **Multipath fallback**: When `allow_multipath` is enabled and no direct
 * path is found, multipath TOF is tracked across beam refinement levels.
 * Convergence is required — two successive beam levels must agree within
 * combined absolute + relative tolerance:
 * @code
 *   |TOF_new - TOF_prev| < kTofConvergenceAtol + kTofConvergenceRtol *
 * |TOF_prev|
 * @endcode
 * Unconverged multipath results are rejected as kNoArrival.
 *
 * The beam count is scaled by kBeamIterativeFactor on each iteration and
 * clamped to `AcousticsBuilder::getMaxBeams()`. After the loop completes,
 * the beam count is restored to its original value for subsequent links.
 *
 * Configuration (via JSON `"acoustics"` block):
 * - `num_beams`: initial beam count per axis (default 80)
 * - `max_beams`: maximum beam count for iterative refinement (default 180)
 * - `beam_spread_deg`: half-cone angle in degrees (default 20.0)
 * - `allow_multipath`: accept converged multipath TOF as fallback (default
 *   false)
 *
 * @see AcousticsBuilder::rebuildBeam(), AcousticsBuilder::getMaxBeams()
 */
class AcousticPairwiseRangeSystem {
public:
  /// @brief Scale factor applied to beam count on each iterative refinement
  /// step.
  static constexpr double kBeamIterativeFactor{2.0};

  /// @brief Absolute TOF convergence tolerance (seconds).
  /// ~15cm range error at 1500 m/s.
  static constexpr double kTofConvergenceAtol{1e-3};

  /// @brief Relative TOF convergence tolerance.
  static constexpr double kTofConvergenceRtol{1e-3};

  /**
   * @brief Constructs the range system.
   *
   * @param builder Acoustics builder that manages source/receiver placement
   * @param context Bellhop context for ray tracing
   * @param mode One-way or two-way TOF scaling
   * @param allowMultipath When true, accept converged multipath TOF as
   *        fallback when no direct path is found
   * @param logAllMeasurements When true, failed measurements are also stored;
   *        when false (default), only successful measurements are logged
   * @param debugRangeErrorPct When > 0, dump ray trace env files for
   *        measurements with range error exceeding this percentage
   * @param debugOutputDir Directory for debug ray trace output files
   */
  AcousticPairwiseRangeSystem(acoustics::AcousticsBuilder &builder,
                              acoustics::BhContext<true, true> &context,
                              GlobalTofMode mode, bool allowMultipath = false,
                              bool logAllMeasurements = false,
                              double debugRangeErrorPct = 0.0,
                              std::string debugOutputDir = "");

  /**
   * @brief Builds full pairwise links with each robot as pinger.
   *
   * @details Creates directed links for:
   * - robot -> every other robot
   * - robot -> every landmark
   *
   * The robot is always assigned as the pinger so that the sound speed
   * profile (SSP) is sampled at the robot's position — the unknown being
   * estimated. For robot-landmark links this means the pinger is the robot
   * even though the physical ping may originate at the landmark.
   *
   * @param world The simulation world containing robots and landmarks
   */
  void rebuildPairs(const rb::RbWorld &world);

  /**
   * @brief Lightweight boundary check that marks out-of-bounds robots as dead.
   *
   * @details Checks each alive robot's position against the acoustic domain
   * boundaries without running Bellhop. Safe to call at physics-rate.
   *
   * @param world The simulation world (robots may be modified)
   */
  void checkBounds(rb::RbWorld &world);

  /**
   * @brief Dumps all necessary files to a text file to run with bellhop tool
   * @details Can rerun these outputs with executable in
   * src/tools/bhc_runner.cpp
   * @param meas Populated measurement details
   * @param link Ranging link
   * @param tag Composite log tag identifying time and endpoints
   * @param simTimeSec Sim Time
   * @param trueRange Actual range between rigid bodies (not bellhops)
   */
  void debugOutputRangeErrors(RangeMeasurement &meas, RangeLink &link,
                              const std::string &tag, double simTimeSec,
                              double trueRange);

  /// @brief Dump a Bellhop env file for offline debugging.
  /// Scales to max beams, sets ray-mode RunType, writes env, then restores.
  void dumpDebugEnv(const RangeLink &link, const std::string &prefix,
                    double simTimeSec);

  /**
   * @brief Runs Bellhop on every active pair and appends measurements to the
   * log.
   *
   * @param simTimeSec Current simulation time in seconds
   * @param world The simulation world (robots may be marked dead if out of
   * bounds)
   */
  void update(double simTimeSec, rb::RbWorld &world);

  /**
   * @brief Returns the flat chronological log of range measurements.
   * @return Const reference to the measurements vector
   */
  [[nodiscard]] const std::vector<RangeMeasurement> &
  getMeasurements() const noexcept;

  /**
   * @brief Returns the current set of directed pairwise links.
   * @return Const reference to the links vector
   */
  [[nodiscard]] const std::vector<RangeLink> &getLinks() const noexcept;

private:
  acoustics::AcousticsBuilder &builder_;
  acoustics::BhContext<true, true> &context_;
  GlobalTofMode mode_{GlobalTofMode::kOneWay};
  bool allowMultipath_{false};
  bool logAllMeasurements_{false};
  double debugRangeErrorPct_{0.0};
  std::string debugOutputDir_;
  std::vector<RangeLink> links_{};
  std::vector<RangeMeasurement> measurements_{};

  /// @brief Append measurement to the log if logAllMeasurements_ is enabled.
  void maybeLog(const RangeMeasurement &meas);

  /// @brief Check if either endpoint is dead and skip the link if so.
  /// @param[in]  world  Simulation world
  /// @param[in]  link   The link to check
  /// @param[out] meas   Measurement to populate with skip status
  /// @return true if the link should be skipped
  bool skipIfDead(const rb::RbWorld &world, const RangeLink &link,
                  RangeMeasurement &meas);

  /// @brief Check if two successive TOF values have converged.
  /// @param curr Current TOF value (must be >= 0)
  /// @param prev Previous TOF value (must be >= 0)
  /// @param[out] delta Populated with |curr - prev| for logging
  /// @return true if |curr - prev| < atol + rtol * |prev|
  static bool checkTofConvergence(float curr, float prev, float &delta);

  /// @brief Acquire time-of-flight for a link via iterative beam refinement.
  /// @details Direct-path arrivals are accepted immediately. When
  /// allowMultipath_ is enabled, multipath arrivals require convergence
  /// across two successive beam levels. See @ref iterative_beam_solver.
  /// @param[in]     link     The acoustic link
  /// @param[in]     tag      Log tag for this measurement
  /// @param[in,out] tofCache Cache of TOF values keyed by unordered robot pair
  /// @return {TOF in seconds, convergence diagnostics}. TOF is negative if
  ///         no arrival found or multipath did not converge.
  std::pair<float, TofConvergenceInfo>
  acquireTof(const RangeLink &link, const std::string &tag,
             std::map<std::pair<size_t, size_t>, float> &tofCache);

  /**
   * @brief Returns the TOF multiplier for the given mode.
   * @param mode One-way (1x) or two-way (2x)
   */
  static float tofScale(GlobalTofMode mode);

  /**
   * @brief Checks whether an endpoint is still alive.
   * @details Landmarks are always considered alive.
   * @param world The simulation world
   * @param endpoint The endpoint to check
   */
  static bool isAlive(const rb::RbWorld &world, const RangeEndpoint &endpoint);

  /**
   * @brief Marks a robot as dead by index.
   * @param world The simulation world
   * @param robotIdx Index into the world's robot list
   */
  static void markRobotDead(rb::RbWorld &world, size_t robotIdx);

  /**
   * @brief Resolves the 3D position of an endpoint.
   * @param world The simulation world
   * @param endpoint The endpoint to resolve
   * @return The position in world coordinates
   */
  static Eigen::Vector3d positionOf(const rb::RbWorld &world,
                                    const RangeEndpoint &endpoint);
};

} // namespace sim
