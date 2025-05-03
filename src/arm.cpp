#include "arm.hpp"
#include "controls.hpp"
#include "lib/util.hpp"

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

    armpos = fmod(armpos, 360.0f);

    float liftPower = liftPID.calculate(armTarget - armpos - initialangle);

    std::cout << armpos << std::endl;

    if (abs(liftPower) > 5 && abs(liftPower) < 20) {
        liftPower = util::sign(liftPower) * std::fmax(abs(liftPower), 20.0);
    }
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