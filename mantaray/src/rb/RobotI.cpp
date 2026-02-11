
#include "rb/RobotI.h"

namespace rb {
RobotI::RobotI(DynamicsBodies &bodies) : bodyIdx_(addDynamicsBody(bodies)) {};
}