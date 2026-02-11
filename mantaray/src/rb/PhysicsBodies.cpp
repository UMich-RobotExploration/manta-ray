//
// Created by tko on 2/9/26.
//

#include "rb/pch.h"

#include "rb/PhysicsBodies.h"
#include "checkAssert.h"
#include "rb/Integrator.h"

namespace rb {

void reserveBodies(DynamicsBodies &bodies, size_t count) {
  bodies.dynamics.reserve(count);
  bodies.kinematics.reserve(count);
  bodies.properties.reserve(count);
}

DynamicsProperties& getDynamicProperties(DynamicsBodies &bodies, BodyIdx index) {
  CHECK(index < bodies.properties.size(), "Index out of bounds for dynamics properties");
  return bodies.properties[index];
}

DynamicsData& getDynamicsData(DynamicsBodies &bodies, BodyIdx index) {
  CHECK(index < bodies.properties.size(), "Index out of bounds for dynamics properties");
  return bodies.dynamics[index];
}
KinematicData& getKinematicData(DynamicsBodies &bodies, BodyIdx index) {
  CHECK(index < bodies.properties.size(), "Index out of bounds for dynamics properties");
  return bodies.kinematics[index];
}
BodyIdx addDynamicsBody(DynamicsBodies &bodies) {
    BodyIdx newIndex = bodies.properties.size();
    bodies.properties.emplace_back(); // Default construct properties
    bodies.dynamics.emplace_back();   // Default construct dynamics data
    bodies.kinematics.emplace_back(); // Default construct kinematic data
    return newIndex;
}


} // namespace rb