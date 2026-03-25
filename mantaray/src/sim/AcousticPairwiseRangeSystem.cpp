#include <mantaray/sim/AcousticPairwiseRangeSystem.h>

#include "acoustics/Arrival.h"
#include "acoustics/helpers.h"
#include "spdlog/spdlog.h"

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

void AcousticPairwiseRangeSystem::update(double simTimeSec,
                                         rb::RbWorld &world) {
  for (auto &link : links_) {
    RangeSample sample;
    sample.simTimeSec = simTimeSec;

    if (!isAlive(world, link.pinger)) {
      sample.status = RangeStatus::kSkippedPingerDead;
      link.samples.push_back(sample);
      continue;
    }
    if (!isAlive(world, link.target)) {
      sample.status = RangeStatus::kSkippedTargetDead;
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
      sample.status = RangeStatus::kOutOfBounds;
      link.samples.push_back(sample);
      continue;
    case acoustics::BoundaryCheck::kSourceOutofBounds:
      if (link.pinger.type == EndpointType::kRobot) {
        markRobotDead(world, link.pinger.index);
      }
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
      sample.status = RangeStatus::kOutOfBounds;
      link.samples.push_back(sample);
      continue;
    }

    bhc::run(context_.params(), context_.outputs());

    acoustics::Arrival arrival(context_.params(), context_.outputs());
    sample.tofRawSec = arrival.getFastestArrival();
    if (sample.tofRawSec < 0.0f) {
      sample.status = RangeStatus::kNoArrival;
      link.samples.push_back(sample);
      continue;
    }

    const auto pos = acoustics::utils::safeEigenToVec23(pingerPos);
    float cPinger = 0.0f;
    bhc::get_ssp<true, true>(context_.params(), pos, cPinger);
    if (cPinger <= 0.0f) {
      sample.status = RangeStatus::kSspSampleFailed;
      link.samples.push_back(sample);
      continue;
    }

    sample.soundSpeedAtPingerMps = cPinger;
    sample.tofEffectiveSec = sample.tofRawSec * tofScale(mode_);
    sample.rangeMeters = sample.tofEffectiveSec * sample.soundSpeedAtPingerMps;
    sample.status = RangeStatus::kOk;
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
