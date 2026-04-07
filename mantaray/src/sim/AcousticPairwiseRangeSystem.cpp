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
    bool logAllMeasurements, double debugRangeErrorPct,
    std::string debugOutputDir)
    : builder_(builder),
      context_(context),
      mode_(mode),
      logAllMeasurements_(logAllMeasurements),
      debugRangeErrorPct_(debugRangeErrorPct),
      debugOutputDir_(std::move(debugOutputDir)) {}

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

void AcousticPairwiseRangeSystem::maybeLog(const RangeMeasurement &meas) {
  if (logAllMeasurements_) {
    measurements_.push_back(meas);
  }
}

bool AcousticPairwiseRangeSystem::skipIfDead(const rb::RbWorld &world,
                                             const RangeLink &link,
                                             RangeMeasurement &meas) {
  if (!isAlive(world, link.pinger)) {
    meas.status = RangeStatus::kSkippedPingerDead;
    SPDLOG_TRACE("Ping dropped: pinger {:d}[{}] is dead", link.pinger.index,
                 link.pinger.type == EndpointType::kRobot ? "robot"
                                                          : "landmark");
    maybeLog(meas);
    return true;
  }
  if (!isAlive(world, link.target)) {
    meas.status = RangeStatus::kSkippedTargetDead;
    SPDLOG_TRACE("Ping dropped: target {:d}[{}] is dead", link.target.index,
                 link.target.type == EndpointType::kRobot ? "robot"
                                                          : "landmark");
    maybeLog(meas);
    return true;
  }
  return false;
}

std::pair<float, TofConvergenceInfo> AcousticPairwiseRangeSystem::acquireTof(
    const RangeLink &link, const std::string &tag,
    std::map<std::pair<size_t, size_t>, float> &tofCache) {
  const bool isRobotPair = link.pinger.type == EndpointType::kRobot &&
                           link.target.type == EndpointType::kRobot;

  if (isRobotPair) {
    auto key = std::make_pair(std::min(link.pinger.index, link.target.index),
                              std::max(link.pinger.index, link.target.index));
    auto it = tofCache.find(key);
    if (it != tofCache.end()) {
      bellhop_logger->debug("{} Using cached TOF (reciprocal)", tag);
      TofConvergenceInfo info{};
      info.fromCache = true;
      info.converged = true;
      info.finalBeams = builder_.getNumBeams();
      return {it->second, info};
    }
  }

  const int originalBeams = builder_.getNumBeams();
  const int maxBeams = builder_.getMaxBeams();
  float tofRawSec = acoustics::kNoArrival;
  float prevTof = -1.0f;
  TofConvergenceInfo info{};
  info.iterations = 0;
  bool tofConverged = false;

  for (int beams = originalBeams; beams <= maxBeams;) {
    ++info.iterations;
    info.finalBeams = beams;

    bellhop_logger->debug("\n===Start Bellhop {} (beams={})===\n", tag, beams);
    if (bellhop_logger->level() == spdlog::level::debug) {
      bhc::echo(context_.params());
    }
    bhc::run(context_.params(), context_.outputs());
    bellhop_logger->debug("\n===End Bellhop {}===\n", tag);

    acoustics::Arrival arrival(context_.params(), context_.outputs());
    tofRawSec = arrival.getFastestArrival(true);

    if (tofRawSec >= 0.0f && prevTof >= 0.0f) {
      // Two successive TOF values — check convergence
      float delta = std::abs(tofRawSec - prevTof);
      info.lastDelta = delta;
      float tolerance = static_cast<float>(
          kTofConvergenceAtol + kTofConvergenceRtol * std::abs(prevTof));
      if (delta < tolerance) {
        SPDLOG_INFO("{} TOF converged: delta={:.2e}s tol={:.2e}s after {} "
                    "iterations (beams={})",
                    tag, delta, tolerance, info.iterations, beams);
        tofConverged = true;
        break;
      }
      SPDLOG_INFO("{} TOF delta={:.2e}s > tol={:.2e}s, not converged", tag,
                  delta, tolerance);
    }

    if (tofRawSec >= 0.0f) {
      prevTof = tofRawSec;
    }

    // Scale up for next iteration
    int nextBeams = static_cast<int>(beams * kBeamIterativeFactor);
    nextBeams = std::min(nextBeams, maxBeams);

    if (beams >= maxBeams) {
      if (prevTof >= 0.0f) {
        SPDLOG_WARN("{} TOF did not converge at max {} beams", tag, maxBeams);
      } else {
        SPDLOG_INFO("{} No direct path found at max {} beams", tag, maxBeams);
      }
      break;
    }

    if (tofRawSec < 0.0f) {
      SPDLOG_INFO("{} No direct path at {} beams, retrying with {}", tag, beams,
                  nextBeams);
    } else {
      SPDLOG_INFO("{} TOF={:.6f}s at {} beams, verifying with {}", tag,
                  tofRawSec, beams, nextBeams);
    }

    builder_.rebuildBeam(nextBeams);
    beams = nextBeams;
  }

  // Restore original beam count
  if (builder_.getNumBeams() != originalBeams) {
    builder_.rebuildBeam(originalBeams);
  }

  // Reject unconverged results
  if (!tofConverged) {
    tofRawSec = acoustics::kNoArrival;
  }
  info.converged = tofConverged;

  if (isRobotPair) {
    auto key = std::make_pair(std::min(link.pinger.index, link.target.index),
                              std::max(link.pinger.index, link.target.index));
    tofCache[key] = tofRawSec;
  }

  return {tofRawSec, info};
}

void AcousticPairwiseRangeSystem::debugOutputRangeErrors(RangeMeasurement &meas,
                                                         RangeLink &link,
                                                         const std::string &tag,
                                                         double simTimeSec,
                                                         double trueRange) {
  // Debug ray trace on high error
  if (debugRangeErrorPct_ > 0.0 && trueRange > 0.0) {
    double errorPct =
        std::abs(meas.rangeMeters - trueRange) / trueRange * 100.0;
    if (errorPct > debugRangeErrorPct_) {
      SPDLOG_WARN("{} Error {:.1f}% exceeds threshold {:.1f}%, "
                  "re-running ray trace",
                  tag, errorPct, debugRangeErrorPct_);

      // Switch to ray trace mode, preserving all other RunType flags
      char savedRunType[7];
      std::strcpy(savedRunType, context_.params().Beam->RunType);
      savedRunType[0] = 'R';
      std::strcpy(context_.params().Beam->RunType, savedRunType);
      savedRunType[0] = 'A'; // prepare restore value

      // Ray Preprocess allocates ray data alongside existing arrivals data.
      // Both fit in memory as long as maxMemory is sufficient.
      bhc::run(context_.params(), context_.outputs());

      auto filename =
          fmt::format("{}/debug_{}{}_to_{}{}_{:.0f}s", debugOutputDir_,
                      link.pinger.type == EndpointType::kRobot ? "R" : "L",
                      link.pinger.index,
                      link.target.type == EndpointType::kRobot ? "R" : "L",
                      link.target.index, simTimeSec);
      // writeenv captures the environment snapshot
      // (source/receiver/SSP/bathy) writeout for ray mode segfaults in
      // bellhop's Ray::Writeout (null ray ptr) NOTE: THis is a fundamental
      // bug in bellhop that we cannot fix
      bhc::writeenv(context_.params(), filename.c_str());

      // Restore arrivals mode — next bhc::run() re-preprocesses for arrivals
      std::strcpy(context_.params().Beam->RunType, savedRunType);
    }
  }
}

void AcousticPairwiseRangeSystem::update(double simTimeSec,
                                         rb::RbWorld &world) {
  // Cache Bellhop TOF for robot-robot pairs — acoustic reciprocity means
  // the propagation time is identical in both directions, so we only need
  // to run Bellhop once per unordered pair.
  std::map<std::pair<size_t, size_t>, float> tofCache;

  int totalLinks = 0;
  int cachedCount = 0;
  int convergedCount = 0;
  int failedCount = 0;

  for (auto &link : links_) {
    RangeMeasurement meas;
    meas.simTimeSec = simTimeSec;
    meas.pinger = link.pinger;
    meas.target = link.target;
    const auto tag = measureTag(simTimeSec, link.pinger, link.target);

    if (skipIfDead(world, link, meas)) {
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
      maybeLog(meas);
      continue;
    }

    // Receiver boundary check
    // Now both source and receiver are set; each case identifies which
    // endpoint(s) to mark dead.
    boundaryCheck = builder_.updateReceiver(targetPos);
    switch (boundaryCheck) {
    case acoustics::BoundaryCheck::kInBounds:
      break;
    case acoustics::BoundaryCheck::kReceiverOutofBounds:
      if (link.target.type == EndpointType::kRobot) {
        markRobotDead(world, link.target.index);
      }
      SPDLOG_WARN("{} Ping dropped: target out of bounds", tag);
      meas.status = RangeStatus::kOutOfBounds;
      maybeLog(meas);
      continue;
    case acoustics::BoundaryCheck::kSourceOutofBounds:
      if (link.pinger.type == EndpointType::kRobot) {
        markRobotDead(world, link.pinger.index);
      }
      SPDLOG_WARN("{} Ping dropped: pinger out of bounds", tag);
      meas.status = RangeStatus::kOutOfBounds;
      maybeLog(meas);
      continue;
    case acoustics::BoundaryCheck::kEitherOrOutOfBounds:
      if (link.target.type == EndpointType::kRobot) {
        markRobotDead(world, link.target.index);
      }
      if (link.pinger.type == EndpointType::kRobot) {
        markRobotDead(world, link.pinger.index);
      }
      SPDLOG_WARN("{} Ping dropped: both pinger and target out of bounds", tag);
      meas.status = RangeStatus::kOutOfBounds;
      maybeLog(meas);
      continue;
    }

    // TOF acquisition with convergence verification
    auto [tofRawSec, convergence] = acquireTof(link, tag, tofCache);
    ++totalLinks;
    if (convergence.fromCache) {
      ++cachedCount;
    } else if (convergence.converged) {
      ++convergedCount;
    } else {
      ++failedCount;
    }

    if (tofRawSec < 0.0f) {
      meas.status = RangeStatus::kNoArrival;
      SPDLOG_WARN("{} Ping dropped: no arrival", tag);
      maybeLog(meas);
      continue;
    }

    // SSP query at pinger position
    const auto pos = acoustics::utils::safeEigenToVec23(pingerPos);
    float cPinger = 0.0f;
    bhc::get_ssp<true, true>(context_.params(), pos, cPinger);
    if (cPinger <= 0.0f) {
      meas.status = RangeStatus::kSspSampleFailed;
      SPDLOG_WARN("{} Ping dropped: SSP sample failed at pinger", tag);
      maybeLog(meas);
      continue;
    }

    // Successful measurement: compute range from TOF and local SSP
    meas.soundSpeedAtPingerMps = cPinger;
    meas.tofEffectiveSec = tofRawSec * tofScale(mode_);
    meas.rangeMeters = meas.tofEffectiveSec * meas.soundSpeedAtPingerMps;
    meas.status = RangeStatus::kOk;
    double trueRange = (pingerPos - targetPos).norm();
    SPDLOG_INFO("{} Ping OK: range={:.2f}m true={:.2f}m err={:.2f}m "
                "tof={:.6f}s ssp={:.1f}m/s",
                tag, meas.rangeMeters, trueRange, meas.rangeMeters - trueRange,
                meas.tofEffectiveSec, meas.soundSpeedAtPingerMps);

    measurements_.push_back(meas);
    debugOutputRangeErrors(meas, link, tag, simTimeSec, trueRange);
  }

  SPDLOG_INFO("t={:.1f}s TOF summary: {} links, {} cached, {} converged, "
              "{} failed",
              simTimeSec, totalLinks, cachedCount, convergedCount, failedCount);
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
