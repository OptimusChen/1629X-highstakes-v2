#pragma once

#include "robot.hpp"

namespace lib::opcontrol {
    void arcade(Robot instance, float turnStrength, float forwardStrength);
    void tank(Robot instance, float left, float right);
    float curve(float val);
}