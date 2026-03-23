#include "CurrentDriftRobot.h"

#include <algorithm>

namespace robots {

manif::SE3Tangentd
CurrentDriftRobot::computeLocalTwist(const rb::DynamicsBodies &bodies,
                                     [[maybe_unused]] double simTime,
                                     double dt) {
  auto twist = manif::SE3Tangentd().setZero();

  if (verticalSpeed_ < 0.0) {
    throw std::invalid_argument("verticalSpeed must be non-negative");
  }

  // Current GridVec interpolation expects (x,y,z).
  const auto &poseGlobal = bodies.kinematics.at(bodyIdx_).poseGlobal;
  const Eigen::Vector3d pos = poseGlobal.translation();

  Eigen::Vector3d currGlobal =
      currentGrid_.interpolateDataValue(pos.x(), pos.y(), pos.z());

  // Dive schedule controls Z (depth). Keep current drift in X/Y.
  // Convention assumed: z increases with depth (+down).
  double vzCmd = 0.0;
  switch (phase_) {
  case Phase::kDescend:
    if (pos.z() >= targetDepth_) {
      phase_ = Phase::kHoldDepth;
      phaseElapsed_ = 0.0;
      vzCmd = 0.0;
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
      phase_ = Phase::kAscend;
    }
    break;
  case Phase::kAscend:
    if (pos.z() <= surfaceDepth_) {
      phase_ = Phase::kHoldSurface;
      phaseElapsed_ = 0.0;
      vzCmd = 0.0;
    } else {
      // P controller on depth to avoid overshoot; saturate to max
      // verticalSpeed_
      const double depthErr = surfaceDepth_ - pos.z(); // negative if too deep
      // Command upward (negative) velocity
      vzCmd = std::clamp(kDepthKp * depthErr, -verticalSpeed_, 0.0);
    }
    break;
  case Phase::kHoldSurface:
    phaseElapsed_ += dt;
    vzCmd = 0.0;
    if (phaseElapsed_ >= surfaceHoldSeconds_) {
      phase_ = Phase::kDescend;
    }
    break;
  default:
    vzCmd = 0.0;
    break;
  }

  currGlobal.z() = vzCmd;

  // RobotI::computeLocalTwist() is expected to return a LOCAL/body-frame twist.
  // Convert the global/world-frame current into the body frame.
  const Eigen::Matrix3d Rwb = poseGlobal.rotation();
  twist.lin() = Rwb.transpose() * currGlobal;
  return twist;
}

} // namespace robots
