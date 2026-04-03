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
  kOk,                ///< Successful measurement
  kSkippedPingerDead, ///< Pinger was already marked dead
  kSkippedTargetDead, ///< Target was already marked dead
  kOutOfBounds,       ///< One or both endpoints outside the acoustic domain
  kNoArrival,         ///< Bellhop produced no valid arrival
  kSspSampleFailed,   ///< Sound speed profile query returned invalid value
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
  EndpointType type{
      EndpointType::kRobot}; ///< Whether this is a robot or landmark
  size_t index{0};           ///< Index into the world's robot or landmark list
};

/**
 * @brief Self-contained record of a single acoustic range measurement.
 *
 * @details Stores everything needed for post-processing, plotting, and factor
 * graph construction (gtsam / PyFactorGraphs) without requiring access to the
 * originating link or simulation state.
 */
struct RangeMeasurement {
  double simTimeSec{0.0}; ///< Simulation time when the measurement was taken
  RangeEndpoint pinger{}; ///< Endpoint that emitted the acoustic ping
  RangeEndpoint target{}; ///< Endpoint that received the acoustic ping
  RangeStatus status{
      RangeStatus::kNoArrival};        ///< Outcome of the measurement attempt
  float rangeMeters{kInvalidDistance}; ///< Computed range in meters
                                       ///< (kInvalidDistance if invalid)
  float tofEffectiveSec{
      kInvalidDistance}; ///< Effective time-of-flight after mode scaling
  float soundSpeedAtPingerMps{
      kInvalidDistance}; ///< Sound speed sampled at the pinger position
};

/**
 * @brief Directed link between two endpoints.
 *
 * @details Used internally to track which pairs to run Bellhop on.
 */
struct RangeLink {
  RangeEndpoint pinger{}; ///< Source of the acoustic ping
  RangeEndpoint target{}; ///< Receiver of the acoustic ping
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
 */
class AcousticPairwiseRangeSystem {
public:
  /**
   * @brief Constructs the range system.
   *
   * @param builder Acoustics builder that manages source/receiver placement
   * @param context Bellhop context for ray tracing
   * @param mode One-way or two-way TOF scaling
   * @param logAllMeasurements When true, failed measurements are also stored;
   *        when false (default), only successful measurements are logged
   */
  AcousticPairwiseRangeSystem(acoustics::AcousticsBuilder &builder,
                              acoustics::BhContext<true, true> &context,
                              GlobalTofMode mode,
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
   * @param simTimeSec Sim Time
   * @param trueRange Actual range between rigid bodies (not bellhops)
   */
  void debugOutputRangeErrors(RangeMeasurement &meas, RangeLink &link,
                              const std::string &tag, double simTimeSec,
                              double trueRange);

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

  /// @brief Acquire time-of-flight for a link, using reciprocal cache when
  /// possible.
  /// @param[in]     link     The acoustic link
  /// @param[in]     tag      Log tag for this measurement
  /// @param[in,out] tofCache Cache of TOF values keyed by unordered robot pair
  /// @return Raw TOF in seconds, or negative if no arrival
  float acquireTof(const RangeLink &link, const std::string &tag,
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
