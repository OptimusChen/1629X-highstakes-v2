#pragma once

#include "lib/controller/pid.hpp"
#include "lib/subsystem.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "robodash/views/selector.hpp"

using namespace pros;
using namespace lib;

#define REST 0
#define LOAD 13
#define MID 40
#define SCORE 180
#define ALLIANCE_STAKE 180

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