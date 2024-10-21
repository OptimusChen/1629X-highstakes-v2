#pragma once

#include "pose.hpp"
#include "odometry/odom.hpp"
#include "lib/pid.hpp"
#include "pros/motor_group.hpp"

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

            // movement stuff
            void turnToHeading(float heading, int timeout);
            void moveToPoint(float x, float y, int timeout, bool forwards = true, bool turnFirst = false);
    };
}