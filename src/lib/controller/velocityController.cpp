#include "lib/controller/velocityController.hpp"
#include "lib/util.hpp"

using namespace lib;

VelocityController::VelocityController(Robot* robot, float dt, double kP, double kI, double kD) {
    this->robot = robot;
    this->dt = dt;

    this->controller = PID(kP, kI, kD);
    this->last = Pose(0, 0, 0);
}

float VelocityController::get_voltage(double desiredVelocity) {
    Pose current = robot->get_pose();
    float distance = sqrt(pow(last.x - current.x, 2) + pow(last.y - current.y, 2)) * METERS;
    float velocity = distance / dt;

    float error = desiredVelocity - velocity;

    return controller.calculate(error);
}