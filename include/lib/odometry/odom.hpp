#pragma once

#include "trackingWheel.hpp"
#include "pros/imu.hpp"
#include "../pose.hpp"

namespace lib {
    class Odom {
        public:
            TrackingWheel* parallel;
            TrackingWheel* perpendicular;
            Imu* inertial;

            float x = 0, y = 0, theta = 0;
            float prevParallel = 0, prevPerpendicular = 0, prevTheta = 0;

            Odom(TrackingWheel* parallel, TrackingWheel* perpendicular, Imu* inertial);

            void update();
            Pose get_pose();
            void start();
            void set_position(float x, float y, float theta, bool radians=false);
    };
}