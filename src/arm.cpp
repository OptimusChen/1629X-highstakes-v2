#include "arm.hpp"
#include "controls.hpp"

void Arm::initialize() {
    if (initialized) return;
    left = new Motor(ARM_LEFT, MotorGears::green);
    right = new Motor(-ARM_RIGHT, MotorGears::green);
    rotation = new Rotation(LB_ROTATION);

    liftPID = PID(1.5, 0, 0.1);

    liftPID.reset();

    rotation->reset();
    rotation->reset_position();
    initialized = true;
}

void Arm::update() {
    float measure = rotation->get_angle() / 100.0f;
    if (measure > 350) measure = 0;

    float liftPower = liftPID.calculate(armTarget - measure);

    left->move(liftPower);
    right->move(liftPower);
}

void Arm::set_target(int target) {
    armTarget = target;
}

void Arm::move(int power) {
    left->move(power);
    right->move(power);
}

void Arm::stop() {}