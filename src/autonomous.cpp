#include "autonomous.hpp"
#include "lib/util.hpp"
#include "lib/controller/pid.hpp"
#include "paths.hpp"
#include "controls.hpp"
#include "intake.hpp"
#include "arm.hpp"
#include "lib/logging.hpp"
#include "s.hpp"

void worldsautonomous::red_sawp(lemlib::Chassis* chassis, Robot* robot) {
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    chassis->setPose(-59, 12, 214.5);
    Arm * arm = robot->get_subsystem<Arm>();
    Intake * intake = robot->get_subsystem<Intake>();
    intake->arm = arm;

    auto stage_1_stop = [&] {
        return intake->detected_ring(STAGE_2);
    };
    
    intake->color_sort = true;
    intake->set_color(RED);

    bool runningAuton = true;

    Task updates([&]() {
        while (runningAuton) {
            arm->update();
            intake->update();
            delay(1);
        }
    });

    arm->set_target(ALLIANCE_STAKE + 80);

    delay(500);

    chassis->moveToPoint(-17, 23, 3000, {.forwards = false, .maxSpeed = 100});

    bool clamped = false;
    Task clamptask([&] {
        while (true) {
            lemlib::Pose pose = chassis->getPose();
            int distanceFrom00 = sqrt(pow(pose.x + 17, 2) + pow(pose.y - 23, 2));
            if (distanceFrom00 < 5) {
                robot->set_mogo(true);
                clamped = true;
                break;
            } 
            delay(10);
        }
    });

    chassis->waitUntilDone();
    arm->set_target(REST);
    if (!clamped) {
        robot->set_mogo(true);
    }

    chassis->turnToPoint(-8, 36, 800);
    chassis->waitUntilDone();
    intake->forwards();
    chassis->moveToPose(-5, 56, 0, 1900, {.forwards = true});
    chassis->waitUntilDone();
    pros::delay(1400);
    chassis->swingToPoint(-21.5, 48, lemlib::DriveSide::LEFT, 1000, {.forwards = true});
    chassis->waitUntilDone();
    chassis->moveToPoint(-21.5, 48, 1000, {.forwards = true});
    chassis->waitUntilDone();

    runningAuton = false;
}