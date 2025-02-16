#pragma once

#include "lib/robot.hpp"
#include "lemlib/api.hpp"

using namespace lib;

namespace bestautonfr {
    void skills(Robot* robot);
    void red_sawp(Robot* robot);
    void blue_sawp(Robot* robot);
    void rush(Robot* robot, lemlib::Chassis* chassis);
    void casey(Robot* robot, lemlib::Chassis* chassis);
    void blue_positive(Robot* robot);
    void red_positive(Robot* robot);
}