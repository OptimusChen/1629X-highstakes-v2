#include "arm.hpp"
#include "controls.hpp"
#include "lib/util.hpp"

void Arm::initialize() {
    if (initialized) return;
    motor = new Motor(LADYBROWN, MotorGears::red);
    motor->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    motor->set_reversed(true);
    rotation = new Rotation(LB_ROTATION);

    liftPID.reset();

    motor->tare_position();
    initialized = true;

    rotation->set_data_rate(5);
}

void Arm::update() {
    if (moving) return;
    if (!rotation) return;
    if (!motor) return;

    float measure = rotation->get_angle() / 100.0f;
    if (measure > 350) measure = 0;
    // float measure = motor->get_position();

    float liftPower = liftPID.calculate(armTarget - measure);

    std::cout << armTarget << " - " << measure << " = " << (armTarget - measure) << std::endl;

    if (abs(liftPower) > 5) {
        liftPower = util::sign(liftPower) * std::fmax(abs(liftPower), 20.0);
    }
    std::cout << liftPower << std::endl;
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