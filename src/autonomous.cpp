#include "autonomous.hpp"
#include "lib/util.hpp"
#include "lib/controller/pid.hpp"
#include "paths.hpp"
#include "controls.hpp"

MPConstraint fast{60, 150, INCH};
MPConstraint slow{30, 150, INCH};

float ret(Rotation rot) {
    float measure = rot.get_angle() / 100.0f;
    if (measure > 350) return 0;
    return measure;
}

void bestautonfr::skills(Robot* robot) {
    float armTarget = 0;

    Motor leftArm(-LEFT_ARM, MotorGears::green);
    Motor rightArm(RIGHT_ARM, MotorGears::green);

    Rotation leftRotation(-LEFT_ROTATION);
    Rotation rightRotation(-RIGHT_ROTATION);

    PID leftLiftPID(1.5, 0, 0.1);
    PID rightLiftPID(1.5, 0, 0.1);

    leftLiftPID.reset();
    rightLiftPID.reset();

    leftRotation.reset();
    rightRotation.reset();
    leftRotation.reset_position();
    rightRotation.reset_position();
    
    Task arm([&]() {
        while (true) {
            float leftPower = leftLiftPID.calculate(armTarget - ret(leftRotation));
            float rightPower = rightLiftPID.calculate(armTarget - ret(rightRotation));

            leftArm.move(leftPower);
            rightArm.move(rightPower);

            delay(10);
        }
    });

    robot->set_pose_mode(MCL);

    pros::delay(3000);
    
    robot->set_mogo(true);
    robot->set_mogo(false);

    // robot->intake(false);
    // delay(2000);
    // robot->stop_intake();

    robot->moveToPoint(-42, 0, 1000, true, false, 60);
    robot->turnToHeading(90, 1000);

    robot->moveToPoint(-42, -25, 3000, false, false, 40);

    delay(500);

    robot->set_mogo(true);

    delay(500);

    robot->turnToHeading(0, 1000);

    robot->intake(false);
    
    robot->ramsete({
        {-47.00, -24.19}, {-13.67, -16.70}, {10.65, -46.10}, {4.47, -53.97},
        {4.47, -53.97}, {-5.59, -66.79}, {-4.40, -40.17}, {-47.20, -47.07},
        {-47.20, -47.07}, {-72.91, -51.22}, {-47.30, -52.89}, {-47.39, -58.71}
    }, fast);

    robot->ramsete({
        {-47.79, -58.71}, {-47.79, -47.60}, {-53.51, -49.04}, {-64.16, -64.62}
    }, fast, false);

    robot->moveToPoint(25, -21.5, 2000, true, true);
    robot->moveToPoint(47, 0, 1000, false, true);

    robot->ramsete({
        {47.07, 0.06}, {47.07, 5.85}, {31.10, 25.90}, {22.62, 24.32},
        {22.62, 24.32}, {14.14, 22.74}, {-5.19, 0.65}, {1.71, -6.64},
        {1.71, -6.64}, {8.61, -13.94}, {40.37, -26.36}, {24.00, -46.68},
        {24.00, -46.68}, {7.63, -66.99}, {25.97, -60.09}, {47.27, -59.69}
    }, fast);

    robot->moveToPoint(27, -60, 1000, false);

    robot->ramsete({
        {27.35, -60.68}, {31.69, -43.52}, {42.73, -47.66}, {46.48, -47.07}
    }, fast);

    return;

    // put in da corner

    robot->ramsete({
        {65.21, -64.82}, {65.21, -56.08}, {39.77, -26.82}, {39.77, -18.08}, 
        {39.77, -18.08}, {39.77, -9.34}, {51.02, 19.78}, {23.01, 47.00}
    }, fast);

    robot->ramsete({
        {23.60, 47.00}, {45.20, 24.02}, {37.80, -0.53}, {59.30, -0.73}
    }, fast, false);

    robot->moveToPoint(60, 24, 2000, false, true);

    // put in da corner

    robot->ramsete({
        {62.65, 62.09}, {36.62, 52.03}, {-21.76, 38.42}, {-23.93, 23.04}
    }, fast);

    robot->moveToPoint(-47.5, 23.5, 2000, false, true);

    robot->ramsete({
        {-47.79, 23.63}, {-47.79, 36.02}, {8.91, 36.35}, {2.01, 49.96}, 
        {2.01, 49.96}, {-4.89, 63.57}, {-19.69, 41.77}, {-56.37, 46.51}, 
        {-56.37, 46.51}, {-64.16, 47.51}, {-56.76, 65.44}, {-47.49, 57.45}, 
    }, fast);

    robot->ramsete({
        {-47.79, 58.71}, {-47.79, 47.60}, {-53.51, 49.04}, {-64.16, 64.62}
    }, fast, false);

    robot->moveToPoint(0, 0, 5000, true, true);

    return;
}   