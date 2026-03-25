#include <mantaray/sim/AcousticPairwiseRangeSystem.h>

#include "acoustics/Arrival.h"
#include "acoustics/helpers.h"
#include "mantaray/utils/Logger.h"

#include <map>

namespace sim {

AcousticPairwiseRangeSystem::AcousticPairwiseRangeSystem(
    acoustics::AcousticsBuilder &builder,
    acoustics::BhContext<true, true> &context, GlobalTofMode mode)
    : builder_(builder), context_(context), mode_(mode) {}

void AcousticPairwiseRangeSystem::rebuildPairs(const rb::RbWorld &world) {
  links_.clear();

  const size_t numRobots = world.robots.size();
  const size_t numLandmarks = world.landmarks.size();

  // Full pairwise with robot as pinger.
  links_.reserve(numRobots * (numRobots > 0 ? numRobots - 1 : 0) +
                 numRobots * numLandmarks);

  for (size_t pingerRobot = 0; pingerRobot < numRobots; ++pingerRobot) {
    for (size_t targetRobot = 0; targetRobot < numRobots; ++targetRobot) {
      if (pingerRobot == targetRobot) {
        continue;
      }
      links_.push_back(RangeLinkState{
          RangeEndpoint{EndpointType::kRobot, pingerRobot},
          RangeEndpoint{EndpointType::kRobot, targetRobot},
          {},
      });
    }

    for (size_t landmarkIdx = 0; landmarkIdx < numLandmarks; ++landmarkIdx) {
      links_.push_back(RangeLinkState{
          RangeEndpoint{EndpointType::kRobot, pingerRobot},
          RangeEndpoint{EndpointType::kLandmark, landmarkIdx},
          {},
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
      SPDLOG_WARN("Robot {:d} out of bounds (boundary check), marking dead", i);
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
    RangeSample sample;
    sample.simTimeSec = simTimeSec;

    const auto *pingerType =
        link.pinger.type == EndpointType::kRobot ? "robot" : "landmark";
    const auto *targetType =
        link.target.type == EndpointType::kRobot ? "robot" : "landmark";

    if (!isAlive(world, link.pinger)) {
      sample.status = RangeStatus::kSkippedPingerDead;
      SPDLOG_WARN("Ping dropped: pinger {:d}[{}] is dead", link.pinger.index,
                  pingerType);
      link.samples.push_back(sample);
      continue;
    }
    if (!isAlive(world, link.target)) {
      sample.status = RangeStatus::kSkippedTargetDead;
      SPDLOG_WARN("Ping dropped: target {:d}[{}] is dead", link.target.index,
                  targetType);
      link.samples.push_back(sample);
      continue;
    }

    const Eigen::Vector3d pingerPos = positionOf(world, link.pinger);
    const Eigen::Vector3d targetPos = positionOf(world, link.target);

    sample.boundaryCheck = builder_.updateSource(pingerPos);
    switch (sample.boundaryCheck) {
    case acoustics::BoundaryCheck::kInBounds:
      break;
    case acoustics::BoundaryCheck::kSourceOutofBounds:
    case acoustics::BoundaryCheck::kReceiverOutofBounds:
    case acoustics::BoundaryCheck::kEitherOrOutOfBounds:
      if (link.pinger.type == EndpointType::kRobot) {
        markRobotDead(world, link.pinger.index);
      }
      SPDLOG_WARN("Ping dropped: pinger {:d}[{}] out of bounds",
                  link.pinger.index, pingerType);
      sample.status = RangeStatus::kOutOfBounds;
      link.samples.push_back(sample);
      continue;
    }

    sample.boundaryCheck = builder_.updateReceiver(targetPos);
    switch (sample.boundaryCheck) {
    case acoustics::BoundaryCheck::kInBounds:
      break;
    case acoustics::BoundaryCheck::kReceiverOutofBounds:
      if (link.target.type == EndpointType::kRobot) {
        markRobotDead(world, link.target.index);
      }
      SPDLOG_WARN("Ping dropped: target {:d}[{}] out of bounds",
                  link.target.index, targetType);
      sample.status = RangeStatus::kOutOfBounds;
      link.samples.push_back(sample);
      continue;
    case acoustics::BoundaryCheck::kSourceOutofBounds:
      if (link.pinger.type == EndpointType::kRobot) {
        markRobotDead(world, link.pinger.index);
      }
      SPDLOG_WARN("Ping dropped: pinger {:d}[{}] out of bounds",
                  link.pinger.index, pingerType);
      sample.status = RangeStatus::kOutOfBounds;
      link.samples.push_back(sample);
      continue;
    case acoustics::BoundaryCheck::kEitherOrOutOfBounds:
      if (link.target.type == EndpointType::kRobot) {
        markRobotDead(world, link.target.index);
      }
      if (link.pinger.type == EndpointType::kRobot) {
        markRobotDead(world, link.pinger.index);
      }
      SPDLOG_WARN("Ping dropped: both pinger {:d}[{}] and target {:d}[{}] out "
                  "of bounds",
                  link.pinger.index, pingerType, link.target.index, targetType);
      sample.status = RangeStatus::kOutOfBounds;
      link.samples.push_back(sample);
      continue;
    }

    // Check TOF cache for robot-robot pairs (acoustic reciprocity)
    const bool isRobotPair = link.pinger.type == EndpointType::kRobot &&
                             link.target.type == EndpointType::kRobot;
    bool tofCached = false;

    if (isRobotPair) {
      auto key = std::make_pair(std::min(link.pinger.index, link.target.index),
                                std::max(link.pinger.index, link.target.index));
      auto it = tofCache.find(key);
      if (it != tofCache.end()) {
        sample.tofRawSec = it->second;
        tofCached = true;
        bellhop_logger->debug(
            "Using cached TOF for robot {:d} -> robot {:d} (reciprocal)",
            link.pinger.index, link.target.index);
      }
    }

    if (!tofCached) {
      bellhop_logger->debug(
          "\n===Start Bellhop Run (pinger {:d}[{}] -> target {:d}[{}])===\n",
          link.pinger.index, pingerType, link.target.index, targetType);
      if (bellhop_logger->level() == spdlog::level::debug) {
        bhc::echo(context_.params());
      }
      bhc::run(context_.params(), context_.outputs());
      bellhop_logger->debug(
          "\n===End Bellhop Run (pinger {:d}[{}] -> target {:d}[{}])===\n",
          link.pinger.index, pingerType, link.target.index, targetType);

      acoustics::Arrival arrival(context_.params(), context_.outputs());
      sample.tofRawSec = arrival.getFastestArrival();

      if (isRobotPair) {
        auto key =
            std::make_pair(std::min(link.pinger.index, link.target.index),
                           std::max(link.pinger.index, link.target.index));
        tofCache[key] = sample.tofRawSec;
      }
    }

    if (sample.tofRawSec < 0.0f) {
      sample.status = RangeStatus::kNoArrival;
      SPDLOG_WARN("Ping dropped: no arrival from {:d}[{}] -> {:d}[{}]",
                  link.pinger.index, pingerType, link.target.index, targetType);
      link.samples.push_back(sample);
      continue;
    }

    const auto pos = acoustics::utils::safeEigenToVec23(pingerPos);
    float cPinger = 0.0f;
    bhc::get_ssp<true, true>(context_.params(), pos, cPinger);
    if (cPinger <= 0.0f) {
      sample.status = RangeStatus::kSspSampleFailed;
      SPDLOG_WARN("Ping dropped: SSP sample failed at pinger {:d}[{}]",
                  link.pinger.index, pingerType);
      link.samples.push_back(sample);
      continue;
    }

    sample.soundSpeedAtPingerMps = cPinger;
    sample.tofEffectiveSec = sample.tofRawSec * tofScale(mode_);
    sample.rangeMeters = sample.tofEffectiveSec * sample.soundSpeedAtPingerMps;
    sample.status = RangeStatus::kOk;
    SPDLOG_INFO("Ping OK: {:d}[{}] -> {:d}[{}] range={:.2f}m tof={:.6f}s "
                "ssp={:.1f}m/s",
                link.pinger.index, pingerType, link.target.index, targetType,
                sample.rangeMeters, sample.tofEffectiveSec,
                sample.soundSpeedAtPingerMps);
    link.samples.push_back(sample);
  }
}

const std::vector<RangeLinkState> &
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
}

bool AcousticPairwiseRangeSystem::isAlive(const rb::RbWorld &world,
                                          const RangeEndpoint &endpoint) {
  switch (endpoint.type) {
  case EndpointType::kRobot:
    return world.robots.at(endpoint.index)->isAlive_;
  case EndpointType::kLandmark:
    return true;
  }
}

void AcousticPairwiseRangeSystem::markRobotDead(rb::RbWorld &world,
                                                size_t robotIdx) {
  if (robotIdx >= world.robots.size()) {
    return;
  }
  auto &robot = world.robots[robotIdx];
  if (robot->isAlive_) {
    SPDLOG_INFO(
        "Marking robot {} as dead due to out-of-bounds acoustic endpoint",
        robotIdx);
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
}

} // namespace sim
