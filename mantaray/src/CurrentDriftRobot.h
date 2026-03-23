#pragma once

#include "acoustics/Grid.h"
#include "rb/RbInterfaces.h"

namespace robots {

/**
 * @brief Robot whose commanded twist is determined by the water current.
 *
 * @details This robot is a simple "drifter": it queries the provided current
 * GridVec at the robot's current global position and uses that as a linear
 * velocity command (angular velocity is zero).
 *
 * The current grid lifetime is managed externally (e.g., by the world/builder).
 * This class stores a non-owning reference.
 */
class CurrentDriftRobot final : public rb::RobotI {
public:
  explicit CurrentDriftRobot(const acoustics::GridVec &currentGrid)
      : currentGrid_(currentGrid) {}

  manif::SE3Tangentd
  computeLocalTwist(const rb::DynamicsBodies &bodies) override;

private:
  const acoustics::GridVec &currentGrid_;
};

} // namespace robots
