//
// Created by tko on 2/9/26.
//

#pragma once

#include "manif/manif.h"

namespace rb {
// Typedef for 6D Vector (twist, wrench, and combined accelerations)
typedef Eigen::Matrix<double, 6, 1> Vector6d;

typedef size_t BodyIdx;

struct DynamicsProperties {
  Eigen::Matrix3d massMatrix = Eigen::Matrix3d::Zero();
  double mass = -100.0;
};
struct DynamicsData {
  Vector6d wrench = Vector6d::Zero();
  Vector6d accel = Vector6d::Zero();
};
struct KinematicData {
  manif::SE3d poseGlobal = manif::SE3d::Identity();
  manif::SE3Tangentd twistLocal = manif::SE3Tangentd::Zero();
};

struct DynamicsBodies {
  std::vector<DynamicsData> dynamics;
  std::vector<KinematicData> kinematics;
  std::vector<DynamicsProperties> properties;
};

void reserveBodies(DynamicsBodies &bodies, size_t count);

DynamicsProperties& getDynamicProperties(DynamicsBodies &bodies, BodyIdx index);
DynamicsData& getDynamicsData(DynamicsBodies &bodies, BodyIdx index);
KinematicData& getKinematicData(DynamicsBodies &bodies, BodyIdx index);
BodyIdx addDynamicsBody(DynamicsBodies &bodies);
}