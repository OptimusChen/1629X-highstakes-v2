#pragma once

#include "lib/robot.hpp"
#include "lemlib/api.hpp"

using namespace lib;

namespace bestautonfr {
    void skills(Robot* robot);
    void sawp(Robot* robot);
    void rush(Robot* robot);
    void casey(Robot* robot, lemlib::Chassis* chassis);
}