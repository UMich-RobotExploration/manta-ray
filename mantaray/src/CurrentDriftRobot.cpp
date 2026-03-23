#include "CurrentDriftRobot.h"

namespace robots {

manif::SE3Tangentd
CurrentDriftRobot::computeLocalTwist(const rb::DynamicsBodies &bodies) {
  auto twist = manif::SE3Tangentd().setZero();

  // Current GridVec interpolation expects (x,y,z).
  const auto &poseGlobal = bodies.kinematics.at(bodyIdx_).poseGlobal;
  const Eigen::Vector3d pos = poseGlobal.translation();

  const Eigen::Vector3d currGlobal =
      currentGrid_.interpolateDataValue(pos.x(), pos.y(), pos.z());

  // RobotI::computeLocalTwist() is expected to return a LOCAL/body-frame twist.
  // Convert the global/world-frame current into the body frame.
  const Eigen::Matrix3d Rwb = poseGlobal.rotation();
  twist.lin() = Rwb.transpose() * currGlobal;
  return twist;
}

} // namespace robots
