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
#include "pros/distance.hpp"

using namespace pros;

#define ODOM 0
#define MCL 1

#define PARTICLES 200
#define MINIMUM 10

struct MPConstraint {
    float speed;
    float accel;
    int unit;
};

struct SensorOrientation {
    int sensorIndex;
};

// SensorOrientation LEFT_O = {0};
// SensorOrientation RIGHT_O = {1};
// SensorOrientation BACK_O = {2};
// SensorOrientation FRONT_O = {3};

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
            PID* angular_slow;

            loco::ParticleFilter<PARTICLES>* particleFilter;
            std::vector<Subsystem*> subsystems;

            int poseMode = ODOM;
            bool poseSet = false;
            bool useSlowAngular = false;
            bool mogostate = false;

            Robot(Odom* odom, MotorGroup* left, MotorGroup* right, PID* lateral, PID* angular, PID* angular_slow);

            void set_pose(float x, float y, float theta, bool radians=false);
            Pose get_pose();
            void calibrate();
            void set_pf(loco::ParticleFilter<PARTICLES>* particleFilter);
            void set_pose_mode(int mde);
            void set_use_slow_angular(bool value);
            void set_brake_mode(motor_brake_mode_e_t mode);
            void reset_particle_filter(float x, float y);
            void initialize_particle_filter();

            void reset_position(SensorOrientation fwdbck, SensorOrientation leftright, int quadrant);

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
            void turnToHeading(float heading, int timeout, bool reversed = false, int minSpeed = 0, int maxSpeed = 127);
            void turnToPoint(float x, float y, int timeout, bool reversed = false, int minSpeed = 0, int maxSpeed = 127);
            void moveToPoint(float x, float y, int timeout, bool forwards = true, bool turnFirst = false, int maxSpeed = 127, bool noTurn = false);
            void ramsete(std::vector<bezier::Point>, MPConstraint constraint, Direction direction = Direction::FORWARDS);
            void timedMove(int power, int timeout);
            void swingToHeading(float heading, int timeout, int side = 1);
            void shivaan(float x, float y, int timeout, float pct, int maxSpeed = 127);
            void relative(float distance, float maxSpeed, int timeout);
            void moveToDistance(float target, Distance& dist, int timeout, float p);

            void set_mogo(bool value);
            void set_rush_arm_left(bool value);
            void set_rush_arm_right(bool value);
            void set_lift_intake(bool value);
            void set_color_sort_piston(bool value);
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