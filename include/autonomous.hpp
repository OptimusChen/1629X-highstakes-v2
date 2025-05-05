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

    // red pos
    void redPos6Ladder(lemlib::Chassis* chassis, Robot* robot);
    void redPos6Baker(lemlib::Chassis* chassis, Robot* robot);
    void redPos5p1Ladder(lemlib::Chassis* chassis, Robot* robot);
    void redPos5p1Baker(lemlib::Chassis* chassis, Robot* robot);

    // blue pos
    void bluePos6Ladder(lemlib::Chassis* chassis, Robot* robot);
    void bluePos6Baker(lemlib::Chassis* chassis, Robot* robot);
    void bluePos5p1Ladder(lemlib::Chassis* chassis, Robot* robot);
    void bluePos5p1Baker(lemlib::Chassis* chassis, Robot* robot);

    // blue neg
}