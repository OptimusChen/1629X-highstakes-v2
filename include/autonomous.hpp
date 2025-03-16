#pragma once

#include "lib/robot.hpp"
#include "lemlib/api.hpp"

using namespace lib;

namespace bestautonfr {
    void skills(Robot* robot);

    void red_sawp(Robot* robot);
    void blue_sawp(Robot* robot);

    void red_rush(Robot* robot);
    void blue_rush(Robot* robot);
    void casey(Robot* robot);
    void blue_positive(Robot* robot);
    void red_positive(Robot* robot);
    void blue_palliance(Robot* robot);
    void red_palliance(Robot* robot);

    void red_sixringplus(Robot* robot, lemlib::Chassis* chassis);
}