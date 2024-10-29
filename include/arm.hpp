#pragma once

#include "lib/controller/pid.hpp"
#include <unordered_set>
#include "controls.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

using namespace pros;

#define rest 0
#define load 28
#define inter 60
#define limit 115

class Arm {
    public:
        int motorPort;
        int rotPort;

        float armError = 0;
        float armTarget = rest;
        bool useArmPid = true;
        bool canControl = true;
        bool canScore = armTarget != rest;

        Rotation* rotation;
        Motor* arm;

        lib::PID armPid = lib::PID(
            0.8, // kP
            0, // kI
            2 // kD
        );

        Arm(Motor* m, Rotation* r);

        void toggle_l2();
        void hold_l1();
        void release_l1();
        void hold_l2();
        void release_l2();

        void score();
        void prime();
        void with_ring();
        void stow();

        void tick(std::unordered_set<controller_digital_e_t> held);
        float arm_pos();
};