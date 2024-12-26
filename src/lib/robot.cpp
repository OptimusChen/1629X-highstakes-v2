#include "lib/robot.hpp"
#include "lib/util.hpp"
#include "lib/controller/pid.hpp"
#include "lib/controller/feedForward.hpp"
#include "lib/bezier.h"
#include "lib/motionProfiling.hpp"
#include "lib/controller/ramsete.hpp"
#include "controls.hpp"
#include <math.h>
#include <iomanip>

using namespace lib;

ADIDigitalOut mogo(MOGO);

Motor hooks(HOOKS);

Robot::Robot(Odom* odom, MotorGroup* left, MotorGroup* right, PID* lateral, PID* angular) {
    this->odometry = odom;

    this->left = left;
    this->right = right;

    this->lateral = lateral;
    this->angular = angular;
}

void Robot::set_pose(float x, float y, float theta, bool radians) {
    odometry->set_position(x, y, theta, radians);
}

void Robot::set_pose_mode(int mode) {
    this->poseMode = mode;
}

Pose Robot::get_pose() {
    if (this->poseMode == MCL) {
        auto pred = particleFilter->getPrediction();
        float cartesianX = -pred.y() * metre.Convert(inch);
        float cartesianY = pred.x() * metre.Convert(inch);
        return Pose(cartesianX, cartesianY, odometry->get_pose().theta, true);
    }
    return odometry->get_pose();
}

void Robot::calibrate() {
    odometry->inertial->reset(true);

    odometry->start();
}

void Robot::set_constants(float wheelDiameter, int rpm, float mass, float trackWidth, float friction_coef) {
    this->wheelDiameter = wheelDiameter;
    this->rpm = rpm;
    this->mass = mass;
    this->trackWidth = trackWidth;
    this->friction_coef = friction_coef;
}

void Robot::add_subsystem(Subsystem* subsystem) {
    subsystems.push_back(subsystem);
}

Subsystem* Robot::get_subsystem(const std::string& name) {
    for (auto* subsystem : subsystems) {
        if (typeid(*subsystem).name() == name) {
            return subsystem;
        }
    }
    return nullptr;
}

void Robot::set_pf(loco::ParticleFilter<PARTICLES>* particleFilter) {
    this->particleFilter = particleFilter;
}

void Robot::set_mogo(bool value) {
    mogo.set_value(value);
}

void Robot::intake(bool reverse) {
    hooks.move(reverse ? 127 : -127);
}

void Robot::stop_intake() {
    hooks.move(0);
}