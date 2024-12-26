#pragma once

#include "lib/controller/pid.hpp"
#include "lib/subsystem.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

using namespace pros;
using namespace lib;

#define REST_LOAD 0
#define SCORE 18

class Arm : public Subsystem {
public:
    int armTarget;

    Motor* left;
    Motor* right;
    Rotation* rotation;
    PID liftPID = PID(1.5, 0, 0.1);

    void initialize() override;
    void update() override;
    void stop() override;

    void set_target(int target);
    void move(int power);
};