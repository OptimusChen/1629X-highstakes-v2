#pragma once

#include "trackingWheel.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "../pose.hpp"

namespace lib {
    class Odom {
        public:
            TrackingWheel* parallel;
            TrackingWheel* perpendicular;
            Imu* inertial;

            float x = 0, y = 0, theta = 0;
            float prevParallel = 0, prevPerpendicular = 0, prevTheta = 0;

            int rpm;
            float wheelDiameter;
            float trackWidth;
            MotorGroup* left;
            MotorGroup* right;

            Odom(TrackingWheel* parallel, TrackingWheel* perpendicular, Imu* inertial);
            Odom(int rpm, float wheelDiameter, float trackWidth, MotorGroup* leftMotors, MotorGroup* rightMotors, Imu* inertial);

            void update();
            Pose get_pose();
            void start();
            void set_position(float x, float y, float theta, bool radians=false);
            float get_right_encoder_travelled();
            float get_left_encoder_travelled();
    };
}