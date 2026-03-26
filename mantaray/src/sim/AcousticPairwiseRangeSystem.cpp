#include <mantaray/sim/AcousticPairwiseRangeSystem.h>

namespace {
/// Compact composite key for correlating general and bellhop logs.
/// Format: "[t=<time> <pingerIdx>[R|L]-><targetIdx>[R|L]]"
std::string measureTag(double simTimeSec, const sim::RangeEndpoint &pinger,
                       const sim::RangeEndpoint &target) {
  const char p = pinger.type == sim::EndpointType::kRobot ? 'R' : 'L';
  const char t = target.type == sim::EndpointType::kRobot ? 'R' : 'L';
  return fmt::format("[t={:.1f} {}[{}]->{}[{}]]", simTimeSec, pinger.index, p,
                     target.index, t);
}
} // namespace

namespace sim {

AcousticPairwiseRangeSystem::AcousticPairwiseRangeSystem(
    acoustics::AcousticsBuilder &builder,
    acoustics::BhContext<true, true> &context, GlobalTofMode mode,
    bool logAllMeasurements)
    : builder_(builder),
      context_(context),
      mode_(mode),
      logAllMeasurements_(logAllMeasurements) {}

void AcousticPairwiseRangeSystem::rebuildPairs(const rb::RbWorld &world) {
  links_.clear();

  const size_t numRobots = world.robots.size();
  const size_t numLandmarks = world.landmarks.size();

  // Convention: the robot is always the pinger, even for robot-landmark links
  // where the physical ping originates at the landmark. This ensures the SSP
  // is always sampled at the robot's position, which is the unknown being
  // estimated — the landmark position is known a priori.
  links_.reserve(numRobots * (numRobots > 0 ? numRobots - 1 : 0) +
                 numRobots * numLandmarks);

  for (size_t pingerRobot = 0; pingerRobot < numRobots; ++pingerRobot) {
    for (size_t targetRobot = 0; targetRobot < numRobots; ++targetRobot) {
      if (pingerRobot == targetRobot) {
        continue;
      }
      links_.push_back(RangeLink{
          RangeEndpoint{EndpointType::kRobot, pingerRobot},
          RangeEndpoint{EndpointType::kRobot, targetRobot},
      });
    }

    for (size_t landmarkIdx = 0; landmarkIdx < numLandmarks; ++landmarkIdx) {
      links_.push_back(RangeLink{
          RangeEndpoint{EndpointType::kRobot, pingerRobot},
          RangeEndpoint{EndpointType::kLandmark, landmarkIdx},
      });
    }
  }
}

void AcousticPairwiseRangeSystem::checkBounds(rb::RbWorld &world) {
  for (size_t i = 0; i < world.robots.size(); ++i) {
    if (!world.robots[i]->isAlive_) {
      continue;
    }
    const auto bodyIdx = world.robots[i]->getBodyIdx();
    const auto pos = world.dynamicsBodies.getPosition(bodyIdx);
    // Set receiver to the same position so that updateAgents() doesn't
    // produce a false out-of-bounds from a stale receiver position.
    builder_.updateReceiver(pos);
    auto result = builder_.updateSource(pos);
    if (result != acoustics::BoundaryCheck::kInBounds) {
      SPDLOG_WARN("Robot {:d} out of bounds (checkBounds), result={:d}, "
                  "pos=[{}, {}, {}]",
                  i, static_cast<int>(result), pos.x(), pos.y(), pos.z());
      markRobotDead(world, i);
    }
  }
}

void AcousticPairwiseRangeSystem::update(double simTimeSec,
                                         rb::RbWorld &world) {
  // Cache Bellhop TOF for robot-robot pairs — acoustic reciprocity means
  // the propagation time is identical in both directions, so we only need
  // to run Bellhop once per unordered pair.
  std::map<std::pair<size_t, size_t>, float> tofCache;

  for (auto &link : links_) {
    RangeMeasurement meas;
    meas.simTimeSec = simTimeSec;
    meas.pinger = link.pinger;
    meas.target = link.target;

    const auto *pingerType =
        link.pinger.type == EndpointType::kRobot ? "robot" : "landmark";
    const auto *targetType =
        link.target.type == EndpointType::kRobot ? "robot" : "landmark";
    const auto tag = measureTag(simTimeSec, link.pinger, link.target);

    // -- Liveness pre-check --
    if (!isAlive(world, link.pinger)) {
      meas.status = RangeStatus::kSkippedPingerDead;
      SPDLOG_TRACE("Ping dropped: pinger {:d}[{}] is dead", link.pinger.index,
                   pingerType);
      if (logAllMeasurements_) {
        measurements_.push_back(meas);
      }
      continue;
    }
    if (!isAlive(world, link.target)) {
      meas.status = RangeStatus::kSkippedTargetDead;
      SPDLOG_TRACE("Ping dropped: target {:d}[{}] is dead", link.target.index,
                   targetType);
      if (logAllMeasurements_) {
        measurements_.push_back(meas);
      }
      continue;
    }

    const Eigen::Vector3d pingerPos = positionOf(world, link.pinger);
    const Eigen::Vector3d targetPos = positionOf(world, link.target);

    // Source boundary check
    // Any non-kInBounds result means the pinger position is invalid for
    // Bellhop, so we kill the pinger robot and skip the link.
    auto boundaryCheck = builder_.updateSource(pingerPos);
    switch (boundaryCheck) {
    case acoustics::BoundaryCheck::kInBounds:
      break;
    // Source, receiver, or both reported out — all treated the same here
    // because we only set the source; any failure is attributable to the
    // pinger.
    case acoustics::BoundaryCheck::kSourceOutofBounds:
    case acoustics::BoundaryCheck::kReceiverOutofBounds:
    case acoustics::BoundaryCheck::kEitherOrOutOfBounds:
      if (link.pinger.type == EndpointType::kRobot) {
        markRobotDead(world, link.pinger.index);
      }
      SPDLOG_WARN("{} Ping dropped: pinger out of bounds", tag);
      meas.status = RangeStatus::kOutOfBounds;
      if (logAllMeasurements_) {
        measurements_.push_back(meas);
      }
      continue;
    }

    // Receiver boundary check
    // Now both source and receiver are set; each case identifies which
    // endpoint(s) to mark dead.
    boundaryCheck = builder_.updateReceiver(targetPos);
    switch (boundaryCheck) {
    case acoustics::BoundaryCheck::kInBounds:
      break;
    // Only the receiver (target) is out of bounds
    case acoustics::BoundaryCheck::kReceiverOutofBounds:
      if (link.target.type == EndpointType::kRobot) {
        markRobotDead(world, link.target.index);
      }
      SPDLOG_WARN("{} Ping dropped: target out of bounds", tag);
      meas.status = RangeStatus::kOutOfBounds;
      if (logAllMeasurements_) {
        measurements_.push_back(meas);
      }
      continue;
    // Only the source (pinger) is out of bounds
    case acoustics::BoundaryCheck::kSourceOutofBounds:
      if (link.pinger.type == EndpointType::kRobot) {
        markRobotDead(world, link.pinger.index);
      }
      SPDLOG_WARN("{} Ping dropped: pinger out of bounds", tag);
      meas.status = RangeStatus::kOutOfBounds;
      if (logAllMeasurements_) {
        measurements_.push_back(meas);
      }
      continue;
    // Both source and receiver are out of bounds
    case acoustics::BoundaryCheck::kEitherOrOutOfBounds:
      if (link.target.type == EndpointType::kRobot) {
        markRobotDead(world, link.target.index);
      }
      if (link.pinger.type == EndpointType::kRobot) {
        markRobotDead(world, link.pinger.index);
      }
      SPDLOG_WARN("{} Ping dropped: both pinger and target out of bounds", tag);
      meas.status = RangeStatus::kOutOfBounds;
      if (logAllMeasurements_) {
        measurements_.push_back(meas);
      }
      continue;
    }

    // TOF acquisition (with reciprocal caching for robot-robot pairs)
    // For robot-robot links, the TOF is symmetric so we only run Bellhop
    // for the first direction and reuse the result for the reverse.
    const bool isRobotPair = link.pinger.type == EndpointType::kRobot &&
                             link.target.type == EndpointType::kRobot;
    bool tofCached = false;
    float tofRawSec = kInvalidDistance;

    if (isRobotPair) {
      auto key = std::make_pair(std::min(link.pinger.index, link.target.index),
                                std::max(link.pinger.index, link.target.index));
      auto it = tofCache.find(key);
      if (it != tofCache.end()) {
        tofRawSec = it->second;
        tofCached = true;
        bellhop_logger->debug("{} Using cached TOF (reciprocal)", tag);
      }
    }

    // Bellhop ray tracing (skipped when TOF is cached)
    if (!tofCached) {
      bellhop_logger->debug("\n===Start Bellhop {}===\n", tag);
      if (bellhop_logger->level() == spdlog::level::debug) {
        bhc::echo(context_.params());
      }
      bhc::run(context_.params(), context_.outputs());
      bellhop_logger->debug("\n===End Bellhop {}===\n", tag);

      acoustics::Arrival arrival(context_.params(), context_.outputs());
      tofRawSec = arrival.getFastestArrival();

      if (isRobotPair) {
        auto key =
            std::make_pair(std::min(link.pinger.index, link.target.index),
                           std::max(link.pinger.index, link.target.index));
        tofCache[key] = tofRawSec;
      }
    }

    // Arrival validation
    if (tofRawSec < 0.0f) {
      meas.status = RangeStatus::kNoArrival;
      SPDLOG_WARN("{} Ping dropped: no arrival", tag);
      if (logAllMeasurements_) {
        measurements_.push_back(meas);
      }
      continue;
    }

    // SSP query at pinger position
    const auto pos = acoustics::utils::safeEigenToVec23(pingerPos);
    float cPinger = 0.0f;
    bhc::get_ssp<true, true>(context_.params(), pos, cPinger);
    if (cPinger <= 0.0f) {
      meas.status = RangeStatus::kSspSampleFailed;
      SPDLOG_WARN("{} Ping dropped: SSP sample failed at pinger", tag);
      if (logAllMeasurements_) {
        measurements_.push_back(meas);
      }
      continue;
    }

    // Successful measurement: compute range from TOF and local SSP
    meas.soundSpeedAtPingerMps = cPinger;
    meas.tofEffectiveSec = tofRawSec * tofScale(mode_);
    meas.rangeMeters = meas.tofEffectiveSec * meas.soundSpeedAtPingerMps;
    meas.status = RangeStatus::kOk;
    SPDLOG_INFO("{} Ping OK: range={:.2f}m tof={:.6f}s ssp={:.1f}m/s", tag,
                meas.rangeMeters, meas.tofEffectiveSec,
                meas.soundSpeedAtPingerMps);
    measurements_.push_back(meas);
  }
}

const std::vector<RangeMeasurement> &
AcousticPairwiseRangeSystem::getMeasurements() const noexcept {
  return measurements_;
}

const std::vector<RangeLink> &
AcousticPairwiseRangeSystem::getLinks() const noexcept {
  return links_;
}

float AcousticPairwiseRangeSystem::tofScale(GlobalTofMode mode) {
  switch (mode) {
  case GlobalTofMode::kOneWay:
    return 1.0f;
  case GlobalTofMode::kTwoWay:
    return 2.0f;
  }
  throw std::logic_error("Unhandled GlobalTofMode");
}

bool AcousticPairwiseRangeSystem::isAlive(const rb::RbWorld &world,
                                          const RangeEndpoint &endpoint) {
  switch (endpoint.type) {
  case EndpointType::kRobot:
    return world.robots.at(endpoint.index)->isAlive_;
  case EndpointType::kLandmark:
    return true;
  }
  throw std::logic_error("Unhandled EndpointType");
}

void AcousticPairwiseRangeSystem::markRobotDead(rb::RbWorld &world,
                                                size_t robotIdx) {
  if (robotIdx >= world.robots.size()) {
    return;
  }
  auto &robot = world.robots[robotIdx];
  if (robot->isAlive_) {
    const auto pos = world.dynamicsBodies.getPosition(robot->getBodyIdx());
    SPDLOG_WARN("Marking robot {} as dead — position: [{}, {}, {}]", robotIdx,
                pos.x(), pos.y(), pos.z());
  }
  robot->isAlive_ = false;
}

Eigen::Vector3d
AcousticPairwiseRangeSystem::positionOf(const rb::RbWorld &world,
                                        const RangeEndpoint &endpoint) {
  switch (endpoint.type) {
  case EndpointType::kRobot: {
    const auto bodyIdx = world.robots.at(endpoint.index)->getBodyIdx();
    return world.dynamicsBodies.getPosition(bodyIdx);
  }
  case EndpointType::kLandmark:
    return world.landmarks.at(endpoint.index);
  }
  throw std::logic_error("Unhandled EndpointType");
}

} // namespace sim
