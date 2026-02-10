//
// Created by tko on 2/10/26.
//

#pragma once

#include "manif/manif.h"

namespace rb {

/*
 * @brief Integrates the pose of rigid body given twist in body frame
 */
void integratePose(const manif::SE3d &pose, const manif::SE3Tangentd &twist,
                   double dt, manif::SE3d &outputPose);

}