
#include "rb/RobotI.h"

namespace rb {
RobotI::~RobotI() = default;  // Add destructor definition

void RobotI::setDynamicsIndex(BodyIdx idx) {
  bodyIdx_ = idx;
}
} // namespace rb