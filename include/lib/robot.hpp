#pragma once

#include "pose.hpp"
#include "odometry/odom.hpp"
#include "lib/controller/pid.hpp"
#include "lib/controller/velocityController.hpp"
#include "lib/bezier.h"
#include "pros/motor_group.hpp"
#include <vector>

using namespace pros;

namespace lib {
    class Robot {
        public:
            Odom* odometry;

            MotorGroup* left;
            MotorGroup* right;

            PID* lateral;
            PID* angular;

            Robot(Odom* odom, MotorGroup* left, MotorGroup* right, PID* lateral, PID* angular);

            void set_pose(float x, float y, float theta, bool radians=false);
            Pose get_pose();
            void calibrate();

            float wheelDiameter;
            int rpm;
            float mass; // kg
            float trackWidth;
            float friction_coef; // how fast to turning in ramsete

            void set_constants(float wheelDiameter, int rpm, float mass, float trackWidth, float friction_coef);

            // movement stuff
            void turnToHeading(float heading, int timeout);
            void turnToPoint(float x, float y, int timeout);
            void moveToPoint(float x, float y, int timeout, bool forwards = true, bool turnFirst = false);
            void ramsete(std::vector<bezier::Point>, bool forwards=true);
    };
}