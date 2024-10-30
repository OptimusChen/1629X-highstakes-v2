#pragma once

#include "pid.hpp"
#include "../robot.hpp"

namespace lib {
    class VelocityController {
        public:
            PID controller;
            Robot* robot;
            float dt;

            Pose last;

            VelocityController(Robot* robot, float dt, double kP, double kI, double kD);
            VelocityController();

            float get_voltage(double desiredVelocity);
    };
}