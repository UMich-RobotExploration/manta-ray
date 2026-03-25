#pragma once

#include "acoustics/AcousticsBuilder.h"
#include "acoustics/BellhopContext.h"
#include "rb/RbWorld.h"
#include <vector>

namespace sim {

enum class GlobalTofMode { kOneWay, kTwoWay };

enum class RangeStatus {
  kOk,
  kSkippedPingerDead,
  kSkippedTargetDead,
  kOutOfBounds,
  kNoArrival,
  kSspSampleFailed,
};

enum class EndpointType { kRobot, kLandmark };

struct RangeEndpoint {
  EndpointType type{EndpointType::kRobot};
  size_t index{0};
};

struct RangeSample {
  double simTimeSec{0.0};
  RangeStatus status{RangeStatus::kNoArrival};
  acoustics::BoundaryCheck boundaryCheck{acoustics::BoundaryCheck::kInBounds};
  float tofRawSec{-1.0f};
  float tofEffectiveSec{-1.0f};
  float soundSpeedAtPingerMps{-1.0f};
  float rangeMeters{-1.0f};
};

struct RangeLinkState {
  RangeEndpoint pinger{};
  RangeEndpoint target{};
  std::vector<RangeSample> samples{};
};

class AcousticPairwiseRangeSystem {
public:
  AcousticPairwiseRangeSystem(acoustics::AcousticsBuilder &builder,
                              acoustics::BhContext<true, true> &context,
                              GlobalTofMode mode);

  // Build full pairwise links with robot as pinger:
  // - robot -> every other robot
  // - robot -> every landmark
  void rebuildPairs(const rb::RbWorld &world);

  // Lightweight boundary check — marks any out-of-bounds robot as dead
  // without running Bellhop. Safe to call at physics-rate.
  void checkBounds(rb::RbWorld &world);

  // Runs Bellhop on every active pair and appends one sample per pair.
  void update(double simTimeSec, rb::RbWorld &world);

  [[nodiscard]] const std::vector<RangeLinkState> &getLinks() const noexcept;

private:
  acoustics::AcousticsBuilder &builder_;
  acoustics::BhContext<true, true> &context_;
  GlobalTofMode mode_{GlobalTofMode::kOneWay};
  std::vector<RangeLinkState> links_{};

  static float tofScale(GlobalTofMode mode);
  static bool isAlive(const rb::RbWorld &world, const RangeEndpoint &endpoint);
  static void markRobotDead(rb::RbWorld &world, size_t robotIdx);
  static Eigen::Vector3d positionOf(const rb::RbWorld &world,
                                    const RangeEndpoint &endpoint);
};

} // namespace sim
