//
// Created by tko on 2/9/26.
//

#pragma once

#include "manif/manif.h"

namespace rb {
// Typedef for 6D Vector (twist, wrench, and combined accelerations)
typedef Eigen::Matrix<double, 6, 1> Vector6d;

typedef size_t BodyIdx;

struct KinematicData {
  manif::SE3d poseGlobal = manif::SE3d::Identity();
  manif::SE3Tangentd twistLocal = manif::SE3Tangentd::Zero();
};

struct DynamicsBodies {
  std::vector<KinematicData> kinematics;
};

void reserveBodies(DynamicsBodies &bodies, size_t count);

KinematicData &getKinematicData(DynamicsBodies &bodies, BodyIdx index);
/* @brief Details namespace is for private internal use of the rb library
 */
namespace detail {
BodyIdx addDynamicsBody(DynamicsBodies &bodies);
}
} // namespace rb