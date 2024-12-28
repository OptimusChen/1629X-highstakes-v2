#pragma once

#include "pose.hpp"
#include "odometry/odom.hpp"
#include "lib/controller/pid.hpp"
#include "lib/bezier.h"
#include "lib/subsystem.hpp"
#include "pros/motor_group.hpp"
#include "pros/adi.hpp"
#include <vector>
#include "localization/particleFilter.h"

using namespace pros;

#define ODOM 0
#define MCL 1

#define PARTICLES 300
#define MINIMUM 10

struct MPConstraint {
    float speed;
    float accel;
    int unit;
};

namespace lib {
    enum Direction {
        FORWARDS,
        BACKWARDS
    };

    class Robot {
        public:
            Odom* odometry;

            MotorGroup* left;
            MotorGroup* right;

            PID* lateral;
            PID* angular;

            loco::ParticleFilter<PARTICLES>* particleFilter;
            std::vector<Subsystem*> subsystems;

            int poseMode = ODOM;

            Robot(Odom* odom, MotorGroup* left, MotorGroup* right, PID* lateral, PID* angular);

            void set_pose(float x, float y, float theta, bool radians=false);
            Pose get_pose();
            void calibrate();
            void set_pf(loco::ParticleFilter<PARTICLES>* particleFilter);
            void set_pose_mode(int mde);

            void add_subsystem(Subsystem* subsystem);
            Subsystem* get_subsystem(const std::string& name);
            template <typename T>
            T* get_subsystem();

            float wheelDiameter;
            int rpm;
            float mass; // kg
            float trackWidth;
            float friction_coef; // how fast to turning in ramsete

            void set_constants(float wheelDiameter, int rpm, float mass, float trackWidth, float friction_coef);

            // movement stuff
            void turnToHeading(float heading, int timeout, int minSpeed = 0);
            void turnToPoint(float x, float y, int timeout, int minSpeed = 0);
            void moveToPoint(float x, float y, int timeout, bool forwards = true, bool turnFirst = false, int maxSpeed = 127);
            void ramsete(std::vector<bezier::Point>, MPConstraint constraint, Direction direction = Direction::FORWARDS);

            void set_mogo(bool value);
            void intake(bool reverse);
            void stop_intake();
    };

    template <typename T>
    T* Robot::get_subsystem() {
        for (auto* subsystem : subsystems) {
            T* specificSubsystem = dynamic_cast<T*>(subsystem);
            if (specificSubsystem) {
                return specificSubsystem;
            }
        }
        return nullptr;
    }
}