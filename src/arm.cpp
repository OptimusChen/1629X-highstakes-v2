#include "arm.hpp"
#include "controls.hpp"
#include "lib/util.hpp"

float armpos = 0;
float prev = 0;

void Arm::initialize() {
    if (initialized) return;
    motor = new Motor(LADYBROWN, MotorGears::red);
    // motor->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    motor->set_reversed(true);
    rotation = new Rotation(LB_ROTATION);

    liftPID.reset();

    motor->tare_position();
    initialized = true;

    rotation->set_data_rate(5);

    armpos = rotation->get_angle() / 100.0f;
    prev = armpos;
}

void Arm::update() {
    if (!rotation) return;
    if (!motor) return;

    float measure = rotation->get_angle() / 100.0f;
    float delta = measure - prev;
    // Fix for wraparound
    if (delta > 180.0f) delta -= 360.0f;
    else if (delta < -180.0f) delta += 360.0f;
    prev = measure;
    armpos += delta / 3.0f;

    if (moving) return;
    
    // if (measure > 350) measure = 0;
    // float measure = motor->get_position();

    float liftPower = liftPID.calculate(armTarget - armpos);

    std::cout << armTarget << " - " << armpos << " = " << (armTarget - armpos) << std::endl;

    if (abs(liftPower) > 5 && abs(liftPower) < 20) {
        liftPower = util::sign(liftPower) * std::fmax(abs(liftPower), 20.0);
    }
    // std::cout << liftPower << std::endl;
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