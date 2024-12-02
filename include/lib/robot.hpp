#pragma once

#include "pose.hpp"
#include "odometry/odom.hpp"
#include "lib/controller/pid.hpp"
#include "lib/controller/velocityController.hpp"
#include "lib/bezier.h"
#include "pros/motor_group.hpp"
#include "pros/adi.hpp"
#include <vector>
#include "localization/particleFilter.h"

using namespace pros;

#define ODOM 0
#define MCL 1

#define PARTICLES 50

struct MPConstraint {
    float speed;
    float accel;
    int unit;
};

namespace lib {
    class Robot {
        public:
            Odom* odometry;

            MotorGroup* left;
            MotorGroup* right;

            PID* lateral;
            PID* angular;

            loco::ParticleFilter<PARTICLES>* particleFilter;

            int poseMode = ODOM;

            Robot(Odom* odom, MotorGroup* left, MotorGroup* right, PID* lateral, PID* angular);

            void set_pose(float x, float y, float theta, bool radians=false);
            Pose get_pose();
            void calibrate();
            void set_pf(loco::ParticleFilter<PARTICLES>* particleFilter);
            void set_pose_mode(int mde);

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
            void ramsete(std::vector<bezier::Point>, MPConstraint constraint, bool forwards=true);

            void set_arm_pistons(bool value);
            void set_mogo(bool value);
            void intake(bool reverse);
    };
}