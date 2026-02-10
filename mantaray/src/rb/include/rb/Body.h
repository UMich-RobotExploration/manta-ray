//
// Created by tko on 2/9/26.
//

#pragma once

#include <cstdint>

namespace rb {

enum class BodyType : uint8_t { k3DOF = 1, k6DOF = 2 };
enum class IntegratorType : uint8_t {kRK4 = 1 };

// struct style
struct Body3DOF {

};

// class style
class PhysicsBody {
public:
  explicit PhysicsBody(BodyType bodyType, IntegratorType integratorType);

private:
  BodyType bodyType_;
  IntegratorType integratorType_;
};

} // namespace rb
