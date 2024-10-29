#pragma once

#include "robot.hpp"

namespace lib::opcontrol {
    void arcade(Robot instance, float turnStrength, float forwardStrength);
    float curve(float val);
}