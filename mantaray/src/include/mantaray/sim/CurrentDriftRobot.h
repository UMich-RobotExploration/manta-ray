/** @file CurrentDriftRobot.h
 * @brief Current-driven drifter robot implementation
 */

#pragma once

#include "acoustics/Grid.h"
#include "fmt_eigen.h"
#include "mantaray/utils/Logger.h"
#include "rb/RbInterfaces.h"

/** @namespace robots
 * @brief Concrete robot implementations
 */
namespace robots {

/**
 * @brief Configuration for CurrentDriftRobot
 */
struct CurrentDriftConfig {
  Eigen::Vector3d position;
  double targetDepth{50.0};
  double holdSeconds{60.0};
  double surfaceHoldSeconds{60.0};
  double verticalSpeed{0.5};
  double surfaceDepth{0.01};
};

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
  explicit CurrentDriftRobot(const acoustics::GridVec &currentGrid,
                             const CurrentDriftConfig &cfg);

  manif::SE3Tangentd computeLocalTwist(const rb::DynamicsBodies &bodies,
                                       double simTime, double dt) override;

private:
  static constexpr double kDepthKp = 10.0;

  enum class Phase { kDescend, kHoldDepth, kAscend, kHoldSurface };

  const acoustics::GridVec &currentGrid_;

  // Dive schedule parameters
  double targetDepth_{50.0};
  double holdSeconds_{60.0};
  double surfaceHoldSeconds_{60.0};
  double verticalSpeed_{0.5};
  double surfaceDepth_{0.01};

  // State
  Phase phase_{Phase::kDescend};
  double phaseElapsed_{0.0};

  /**
   * @brief Helper for phase transitions.
   * @details Resets per-phase timer and sets the vertical command to zero for
   * the current step.
   */
  void transitionTo(Phase nextPhase, double &vzCmd);
};

} // namespace robots
