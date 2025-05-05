#pragma once

#include "lib/robot.hpp"
#include "lemlib/api.hpp"

using namespace lib;

namespace worldsautonomous {
    void red_sawp(lemlib::Chassis* chassis, Robot* robot);
    void blue_sawp(lemlib::Chassis* chassis, Robot* robot);

    // red neg
    void red6p1CornerClear(lemlib::Chassis* chassis, Robot* robot);
    void red6p1Ladder(lemlib::Chassis* chassis, Robot* robot);
    void red6p1CornerNoSweep(lemlib::Chassis* chassis, Robot* robot);
}