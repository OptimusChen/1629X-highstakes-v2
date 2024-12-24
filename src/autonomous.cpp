#include "autonomous.hpp"
#include "lib/util.hpp"
#include "lib/controller/pid.hpp"
#include "paths.hpp"
#include "controls.hpp"

MPConstraint fast{60, 200, INCH};

float ret(Rotation rot) {
    float measure = rot.get_angle() / 100.0f;
    if (measure > 355) return 0;
    return measure;
}

void bestautonfr::skills(Robot* robot) {
    robot->set_pose_mode(MCL);

    pros::delay(3000);

    robot->ramsete({{60.00, 0.00}, {5.632, 0}, {58.51, -39.64}, {58.51, -47.52}}, fast);

    // pros::delay(3000);

    // robot->moveToPoint(-23.5, 0, 7000, true, false, 20);
    // robot->turnToHeading(270, 7000);
    // robot->moveToPoint(-23.5, -23.5, 10000, true, false, 20);
    // robot->turnToHeading(180, 3000);
    // robot->moveToPoint(-50, -23.5, 4000, true, false, 20);
    // robot->turnToHeading(90, 3000);
    // robot->moveToPoint(-50, -50, 4000, true, false, 20);

    // robot->ramsete({
    //     {-59.42, 0.06}, {-32.80, 0.26}, {-59.42, -55.36}, {0.13, -47.66}, 
    //     {0.13, -47.66}, {35.24, -43.13}, {52.59, -43.32}, {47.07, 0.06},
    //     {47.07, 0.06}, {41.55, 43.45}, {38.59, 51.14}, {-0.26, 47.39}, 
    //     {-0.26, 47.39}, {-39.11, 43.65}, {-51.93, 35.36}, {-47.39, -0.13}
    // }, fast);

    // robot->ramsete({
    //     {0, 0}, {0, 40}, {40, 0}, {40, 40}
    // }, fast, fast);

    return;

    // float REST_LOAD = 0;
    // float SCORE = 300;
    // float armTarget = REST_LOAD;

    // Motor leftArm(-LEFT_ARM, MotorGears::green);
    // Motor rightArm(RIGHT_ARM, MotorGears::green);

    // auto arm_left = ADIDigitalOut(ARM_PISTON_LEFT);
	// auto arm_right = ADIDigitalOut(ARM_PISTON_RIGHT);

    // Rotation leftRotation(-LEFT_ROTATION);
    // Rotation rightRotation(-RIGHT_ROTATION);

    // PID leftLiftPID(1.5, 0, 0.1);
    // PID rightLiftPID(1.5, 0, 0.1);

    // leftLiftPID.reset();
    // rightLiftPID.reset();

    // leftRotation.reset();
    // rightRotation.reset();
    // leftRotation.reset_position();
    // rightRotation.reset_position();
    
    // Task arm([&]() {
    //     while (true) {
    //         float leftPower = leftLiftPID.calculate(armTarget - ret(leftRotation));
    //         float rightPower = rightLiftPID.calculate(armTarget - ret(rightRotation));

    //         leftArm.move(leftPower);
    //         rightArm.move(rightPower);

    //         delay(10);
    //     }
    // });

    // robot->set_pose_mode(MCL);

    // pros::delay(3000);

    // // robot->intake(false);
    // // delay(2000);
    // // robot->stop_intake();

    // robot->moveToPoint(-42, 0, 1000, true, false, 60);
    // robot->turnToHeading(90, 1000);

    // robot->moveToPoint(-42, -25, 3000, false, false, 20);

    // delay(500);

    // robot->set_mogo(true);

    // delay(500);

    // robot->turnToHeading(0, 1000);

    // robot->intake(false);
    
    // robot->ramsete({
    //     {-47.00, -24.19}, {-13.67, -16.70}, {10.65, -46.10}, {4.47, -53.97},
    //     {4.47, -53.97}, {-5.59, -66.79}, {-4.40, -40.17}, {-47.20, -47.07},
    //     {-47.20, -47.07}, {-72.91, -51.22}, {-47.30, -52.89}, {-47.39, -58.71}
    // }, fast);

    // delay(5000);

    // return;

    // robot->ramsete({
    //     {-47.79, -58.71}, {-47.79, -47.60}, {-53.51, -49.04}, {-64.16, -64.62}
    // }, fast, false);
    // return;
}   