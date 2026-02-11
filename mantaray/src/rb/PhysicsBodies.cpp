//
// Created by tko on 2/9/26.
//

#include "include/rb/PhysicsBodies.h"

#include "rb/pch.h"

#include "rb/PhysicsBodies.h"

namespace rb {

PhysicsBody::PhysicsBody(BodyType bodyType, IntegratorType integratorType)
    : bodyType_(bodyType), integratorType_(integratorType) {}

} // namespace rb