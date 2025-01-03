#pragma once

#include "robot.hpp"

namespace lib::opcontrol {
    void arcade(Robot instance, float turnStrength, float forwardStrength);
    void tank(Robot instance, float left, float right);
    void cheeze(Robot instance, float throttle, float turn, float threshold);
    float curve(float val);
}