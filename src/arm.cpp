#include "arm.hpp"
#include "controls.hpp"

void Arm::initialize() {
    if (initialized) return;
    // left = new Motor(ARM_LEFT, MotorGears::green);
    // right = new Motor(-ARM_RIGHT, MotorGears::green);
    motor = new Motor(LADYBROWN, MotorGears::red);
    motor->set_reversed(true);
    // motor->set_encoder_units(MotorEncoderUnits::degrees);
    rotation = new Rotation(LB_ROTATION);

    liftPID = PID(1.5, 0, 0.1);

    liftPID.reset();

    motor->tare_position();
    initialized = true;
}

void Arm::update() {
    if (moving) return;
    float measure = rotation->get_angle() / 100.0f;
    if (measure > 350) measure = 0;
    // float measure = motor->get_position();

    float liftPower = liftPID.calculate(armTarget - measure);

    motor->move(liftPower);
}

void Arm::set_target(int target) {
    armTarget = target;
}

void Arm::move(int power) {
    motor->move(power);
    moving = true;
}

void Arm::stop() {}