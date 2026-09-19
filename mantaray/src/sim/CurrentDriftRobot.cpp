#include "mantaray/sim/CurrentDriftRobot.h"

#include <algorithm>

namespace robots {

void CurrentDriftRobot::transitionTo(Phase nextPhase, double &vzCmd) {
  phase_ = nextPhase;
  phaseElapsed_ = 0.0;
  vzCmd = 0.0;
}

CurrentDriftRobot::CurrentDriftRobot(const acoustics::GridVec &currentGrid,
                                     const CurrentDriftConfig &cfg)
    : currentGrid_(currentGrid),
      targetDepth_(cfg.targetDepth),
      holdSeconds_(cfg.holdSeconds),
      surfaceHoldSeconds_(cfg.surfaceHoldSeconds),
      verticalSpeed_(cfg.verticalSpeed),
      surfaceDepth_(cfg.surfaceDepth),
      startOffsetSeconds_(cfg.startOffsetSeconds) {

  if (verticalSpeed_ < 0.0) {
    throw std::invalid_argument("verticalSpeed must be non-negative");
  }
  if (startOffsetSeconds_ < 0.0) {
    throw std::invalid_argument("startOffsetSeconds must be non-negative");
  }
  if (startOffsetSeconds_ > 0.0) {
    phase_ = Phase::kHoldSurface;
    firstSurfaceHold_ = true;
  }
}
manif::SE3Tangentd
CurrentDriftRobot::computeLocalTwist(const rb::DynamicsBodies &bodies,
                                     [[maybe_unused]] double simTime,
                                     double dt) {
  auto twist = manif::SE3Tangentd().setZero();

  // Current GridVec interpolation expects (x,y,z).
  const auto &poseGlobal = bodies.kinematics.at(bodyIdx_).poseGlobal;
  const Eigen::Vector3d pos = poseGlobal.translation();

  SPDLOG_TRACE("Position: {}", pos);
  Eigen::Vector3d currentGlobalFrame =
      currentGrid_.interpolateDataValue(pos.x(), pos.y(), pos.z());
  SPDLOG_TRACE("Current Global Frame: {}", currentGlobalFrame);

  // Dive schedule controls Z (depth). Keep current drift in X/Y.
  // Convention assumed: z increases with depth (+down).
  double vzCmd = 0.0;
  switch (phase_) {
  case Phase::kDescend:
    if (pos.z() >= targetDepth_) {
      SPDLOG_DEBUG("Transitioning to : kHoldDepth from kDescend");
      transitionTo(Phase::kHoldDepth, vzCmd);
    } else {
      // P controller on depth to avoid overshoot; saturate to max
      // verticalSpeed_
      const double depthErr = targetDepth_ - pos.z(); // positive if too shallow
      vzCmd = std::clamp(kDepthKp * depthErr, 0.0, verticalSpeed_);
    }
    break;
  case Phase::kHoldDepth:
    phaseElapsed_ += dt;
    vzCmd = 0.0;
    if (phaseElapsed_ >= holdSeconds_) {
      SPDLOG_DEBUG("Transitioning to : kAscend from kHoldDepth after: {}",
                   phaseElapsed_);
      transitionTo(Phase::kAscend, vzCmd);
    }
    break;
  case Phase::kAscend:
    if (pos.z() <= surfaceDepth_) {
      SPDLOG_DEBUG("Transitioning to : kHoldSurface from kAscend ");
      transitionTo(Phase::kHoldSurface, vzCmd);
    } else {
      // P controller on depth to avoid overshoot; saturate to max
      // verticalSpeed_
      const double depthErr = surfaceDepth_ - pos.z(); // negative if too deep
      // Command upward (negative) velocity
      vzCmd = std::clamp(kDepthKp * depthErr, -verticalSpeed_, 0.0);
    }
    break;
  case Phase::kHoldSurface: {
    phaseElapsed_ += dt;
    vzCmd = 0.0;
    const double holdLimit =
        firstSurfaceHold_ ? startOffsetSeconds_ : surfaceHoldSeconds_;
    if (phaseElapsed_ >= holdLimit) {
      SPDLOG_DEBUG("Transitioning to : kDescend from kHoldSurface after: {}",
                   phaseElapsed_);
      firstSurfaceHold_ = false;
      transitionTo(Phase::kDescend, vzCmd);
    }
    break;
  }
  default:
    vzCmd = 0.0;
    break;
  }

  currentGlobalFrame.z() = vzCmd;

  // RobotI::computeLocalTwist() is expected to return a LOCAL/body-frame twist.
  // Convert the global/world-frame current into the body frame.
  const Eigen::Matrix3d Rwb = poseGlobal.rotation().transpose();
  twist.lin() = Rwb * currentGlobalFrame;
  SPDLOG_TRACE("Rwb: {}", Rwb);
  SPDLOG_TRACE("Twist Transformed: {}",
               static_cast<Eigen::Vector3d>(twist.lin()));
  return twist;
}

} // namespace robots
