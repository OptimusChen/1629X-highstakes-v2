#include "arm.hpp"
#include "pros/misc.hpp"

Arm::Arm(Motor* m, Rotation* r) {
    this->motorPort = motorPort;
    this->rotPort = rotPort;

    armPid.reset();
    armPid.set_winduprange(5);

    this->rotation = r;
    this->rotation->reset_position();

    this->arm = m;
    this->arm->set_brake_mode(E_MOTOR_BRAKE_HOLD);
}

void Arm::score() {
    armTarget = limit - 1;
}

void Arm::prime() {
    armTarget = load;
}

void Arm::with_ring() {
    armTarget = inter;
}

void Arm::stow() {
    armTarget = rest;
}

// drive control stuff

void Arm::toggle_l2() {
    this->armTarget = rest;
    this->useArmPid = true;
    this->canScore = false;
}

void Arm::hold_l1() {
    if (!canControl) return;
    // if (!canScore) return;
    
    useArmPid = false;
    arm->move(127);
}

void Arm::release_l1() {
    arm->brake();
    useArmPid = true;
    canScore = true;

    if (armTarget == load) {
        armTarget = inter;
    } else {
        armTarget = load;
    }

    if (arm_pos() > inter) {
        armTarget = load;
    } 
}

void Arm::hold_l2() {
    useArmPid = false;
}

void Arm::release_l2() {
    arm->brake();
    armTarget = rest;
    useArmPid = true;
    canScore = false;
}

void Arm::tick(std::unordered_set<controller_digital_e_t> held) {
    // std::cout<< armTarget <<std::endl;

    if (useArmPid == true) {
        armError = armTarget - arm_pos();
        float value = armPid.calculate(armError);
        int sign = int(value/abs(value));

        if (abs(armError) > 70) {
            arm->move(127 * sign);
        } else {
            arm->move(sign < 0 ? value * 0.5 : value * 2);
        }
    }

    // if the arm is at the code limit, try to reset it backwards to stop it
    if (arm_pos() >= limit && (held.contains(E_CONTROLLER_DIGITAL_L1) || held.contains(E_CONTROLLER_DIGITAL_L2))) {
        canControl = false;
        useArmPid = false;

        arm->move_velocity(0);
    } else {
        canControl = true;
    }

    if (arm_pos() < 1 && arm_pos() > 0) {
        armTarget = rest;

        arm->tare_position();
        armPid.reset();
    }
}

float Arm::arm_pos() {
    float measure = rotation->get_position() / 100.0f;

    if (measure > 350 || measure < 10) return 0;

    if (measure < 360) return measure;
    return measure - 360;
}