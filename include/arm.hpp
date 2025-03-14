#pragma once

#include "lib/controller/pid.hpp"
#include "lib/subsystem.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "robodash/views/selector.hpp"

using namespace pros;
using namespace lib;

#define REST 0
#define LOAD 23
#define MID 40
#define SCORE 130
#define ALLIANCE_STAKE 150

class Arm : public Subsystem {
public:
    int armTarget;

    Motor* motor;
    Rotation* rotation;
    PID liftPID = PID(1.5, 0, 0);
    bool moving = false;

    void initialize() override;
    void update() override;
    void stop() override;

    void set_target(int target);
    void move(int power);
};