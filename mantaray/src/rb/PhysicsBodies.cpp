//
// Created by tko on 2/9/26.
//

#include "rb/pch.h"

#include "rb/PhysicsBodies.h"
#include "checkAssert.h"
#include "rb/Integrator.h"

namespace rb {

void reserveBodies(DynamicsBodies &bodies, size_t count) {
  bodies.kinematics.reserve(count);
}


KinematicData& getKinematicData(DynamicsBodies &bodies, BodyIdx index) {
  CHECK(index < bodies.kinematics.size(), "Index out of bounds for dynamics properties");
  return bodies.kinematics[index];
}
BodyIdx addDynamicsBody(DynamicsBodies &bodies) {
    BodyIdx newIndex = bodies.kinematics.size();
    bodies.kinematics.emplace_back(); // Default construct kinematic data
    return newIndex;
}


} // namespace rb