#include "autonomous.hpp"
#include "lib/util.hpp"
#include "lib/controller/pid.hpp"
#include "paths.hpp"
#include "controls.hpp"
#include "intake.hpp"
#include "arm.hpp"
#include "lib/logging.hpp"
#include "s.hpp"

void redNegStart(lemlib::Chassis* chassis, Robot* robot, Arm* arm, Intake* intake) {
    chassis->setPose(-59, 12, 214.5);

    arm->set_target(ALLIANCE_STAKE + 80);

    delay(500);

    chassis->moveToPoint(-17, 23, 3000, {.forwards = false, .maxSpeed = 100});
    intake->forwards();

    bool clamped = false;
    Task clamptask([&] {
        while (true) {
            lemlib::Pose pose = chassis->getPose();
            int distanceFrom00 = sqrt(pow(pose.x + 17, 2) + pow(pose.y - 23, 2));
            if (distanceFrom00 < 5) {
                robot->set_mogo(true);
                clamped = true;
                chassis->cancelMotion();
                break;
            } 
            delay(10);
        }
    });

    chassis->waitUntilDone();

    arm->set_target(MID + 50);
    if (!clamped) {
        robot->set_mogo(true);
    }

    chassis->turnToPoint(-8, 36, 500);
    chassis->waitUntilDone();
    intake->color_sort = false;
    intake->antijam = false;
    chassis->moveToPose(-5, 56, 0, 1200, {.forwards = true, .maxSpeed = 127, .minSpeed = 40});
    chassis->waitUntilDone();
    pros::delay(400);
    robot->right->move(40);
    chassis->swingToPoint(-21.5, 48, lemlib::DriveSide::LEFT, 1000, {.forwards = true, .minSpeed=20, .earlyExitRange=8});
    chassis->waitUntilDone(); 
    intake->color_sort = true;
    intake->antijam = true;              

    chassis->moveToPoint(-21.5, 48, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed=60});
    chassis->waitUntilDone();
    // chassis->moveToPose(-57, 57, 315, 2000, {.forwards = true, .lead=0.2, .maxSpeed=127});
    chassis->moveToPoint(-56, 56, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed=60});
    chassis->waitUntilDone();
    chassis->turnToPoint(-70, 70, 200);
    chassis->waitUntilDone();
}
void worldsautonomous::red_sawp(lemlib::Chassis* chassis, Robot* robot) {
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    Arm * arm = robot->get_subsystem<Arm>();
    Intake * intake = robot->get_subsystem<Intake>();
    intake->arm = arm; 
    
    intake->color_sort = true;
    intake->set_color(RED);

    intake->colorSortOptical->set_integration_time(10);
    intake->colorSortOptical->set_led_pwm(100);   

    auto stage_1_stop = [&] {
        auto opticalMeasure = intake->colorSortOptical->get_hue();
        bool correctColor = true;
        if (intake->color == RED && ((opticalMeasure > 170) && (opticalMeasure < 245))) correctColor = false;
        if (intake->color == BLUE && ((opticalMeasure > 340 && opticalMeasure < 360) || (opticalMeasure < 20))) correctColor = false;
        return intake->detected_ring(STAGE_2) && correctColor;
    };

    bool runningAuton = true;

    Task updates([&]() {
        while (runningAuton) {
            arm->update();
            intake->update();
            delay(1);
        }
    });

    redNegStart(chassis, robot, arm, intake);
 
    intake->color_sort = false;
    intake->antijam = false;    
    chassis->moveToPoint(-70, 70, 500, {.forwards = true, .maxSpeed = 127, .minSpeed=80});  
    chassis->waitUntilDone();   
    
    chassis->moveToPoint(-47, 47, 1000, {.forwards = false, .maxSpeed = 127});
    chassis->waitUntilDone();
    intake->color_sort = true;
    intake->antijam = true; 
    
    robot->set_rush_arm_left(true);
    chassis->turnToPoint(-47, 18, 500);
    chassis->waitUntilDone();
    robot->set_rush_arm_left(false);
    robot->set_lift_intake(true);
    chassis->moveToPose(-47, 12, 180, 1000, {.forwards = true, .lead=0.2, .maxSpeed=127, .minSpeed=60});
    chassis->waitUntilDone();
    chassis->moveToPoint(-47, 6, 1000, {.forwards = true, .maxSpeed = 40});
    chassis->waitUntilDone();

    Task dropGoal([&] {
        delay(300);
        robot->set_lift_intake(false);
        delay(300);
        robot->set_mogo(false);
        delay(100);
        intake->color_sort = false;
        delay(300);
        arm->set_target(ALLIANCE_STAKE - 10);
        intake->forwards(90);
        delay(500);
        intake->set_stop_condition(stage_1_stop);
    });
    chassis->moveToPoint(-20, -47, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed=80});
    chassis->waitUntilDone();

    delay(500);

    chassis->turnToPoint(-20.5, -21, 500, {.forwards = false});
    chassis->waitUntilDone();

    intake->set_stop_condition(nullptr);
    intake->backwards(20);

    chassis->moveToPoint(-20.5, -21, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed=80});
    chassis->waitUntilDone();

    bool clamped = false;
    Task clamptask2([&] {
        while (true) {
            lemlib::Pose pose = chassis->getPose();
            int distanceFrom00 = sqrt(pow(pose.x + 20.5, 2) + pow(pose.y + 21, 2));
            if (distanceFrom00 < 5) {
                robot->set_mogo(true);
                clamped = true;
                chassis->cancelMotion();
                break;
            } 
            delay(10);
        }
    });

    chassis->waitUntilDone();

    if (!clamped) {
        robot->set_mogo(true);
    }

    intake->color_sort = true;
    intake->forwards();
    // delay(750);
    chassis->turnToPoint(-12, -12, 500, {.forwards = true, .maxSpeed = 60, .earlyExitRange = 5});
    chassis->waitUntilDone();
    arm->set_target(ALLIANCE_STAKE + 20);
    robot->timedMove(20, 200);

    runningAuton = false;               
}

void blueNegStart(lemlib::Chassis* chassis, Robot* robot, Arm* arm, Intake* intake) {
    chassis->setPose(57, 12, 145.5);

    arm->set_target(ALLIANCE_STAKE + 80);

    delay(500);

    chassis->moveToPoint(17, 23, 3000, {.forwards = false, .maxSpeed = 100});
    intake->forwards();

    bool clamped = false;
    Task clamptask([&] {
        while (true) {
            lemlib::Pose pose = chassis->getPose();
            int distanceFrom00 = sqrt(pow(pose.x - 17, 2) + pow(pose.y - 23, 2));
            if (distanceFrom00 < 5) {
                robot->set_mogo(true);
                clamped = true;
                chassis->cancelMotion();
                break;
            } 
            delay(10);
        }
    });

    chassis->waitUntilDone();

    arm->set_target(MID + 50);
    if (!clamped) {
        robot->set_mogo(true);
    }

    chassis->turnToPoint(8, 36, 500);
    chassis->waitUntilDone();
    intake->color_sort = false;
    intake->antijam = false;
    chassis->moveToPose(5, 56, 0, 1200, {.forwards = true, .maxSpeed = 127, .minSpeed = 40});
    chassis->waitUntilDone();
    pros::delay(400);
    robot->right->move(40);
    chassis->swingToPoint(21.5, 48, lemlib::DriveSide::RIGHT, 1000, {.forwards = true, .minSpeed=20, .earlyExitRange=8});
    chassis->waitUntilDone(); 
    intake->color_sort = true;
    intake->antijam = true;              

    chassis->moveToPoint(21.5, 48, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed=60});
    chassis->waitUntilDone();
    // chassis->moveToPose(-57, 57, 315, 2000, {.forwards = true, .lead=0.2, .maxSpeed=127});
    chassis->moveToPoint(56, 56, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed=60});
    chassis->waitUntilDone();
    chassis->turnToPoint(70, 70, 200);
    chassis->waitUntilDone();
}

void worldsautonomous::blue_sawp(lemlib::Chassis* chassis, Robot* robot) {
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    Arm * arm = robot->get_subsystem<Arm>();
    Intake * intake = robot->get_subsystem<Intake>();
    intake->arm = arm; 
    
    intake->color_sort = true;
    intake->set_color(BLUE);

    intake->colorSortOptical->set_integration_time(10);
    intake->colorSortOptical->set_led_pwm(100);   

    auto stage_1_stop = [&] {
        auto opticalMeasure = intake->colorSortOptical->get_hue();
        bool correctColor = true;
        if (intake->color == RED && ((opticalMeasure > 170) && (opticalMeasure < 245))) correctColor = false;
        if (intake->color == BLUE && ((opticalMeasure > 340 && opticalMeasure < 360) || (opticalMeasure < 20))) correctColor = false;
        return intake->detected_ring(STAGE_2) && correctColor;
    };

    bool runningAuton = true;

    Task updates([&]() {
        while (runningAuton) {
            arm->update();
            intake->update();
            delay(1);
        }
    });

    blueNegStart(chassis, robot, arm, intake);
 
    intake->color_sort = false;
    intake->antijam = false;    
    chassis->moveToPoint(70, 70, 500, {.forwards = true, .maxSpeed = 127, .minSpeed=80});  
    chassis->waitUntilDone();   
    
    chassis->moveToPoint(47, 47, 1000, {.forwards = false, .maxSpeed = 127});
    chassis->waitUntilDone();
    intake->color_sort = true;
    intake->antijam = true; 
    
    robot->set_rush_arm_right(true);
    chassis->turnToPoint(47, 18, 500);
    chassis->waitUntilDone();
    robot->set_rush_arm_right(false);
    robot->set_lift_intake(true);
    chassis->moveToPose(47, 12, 180, 1000, {.forwards = true, .lead=0.2, .maxSpeed=127, .minSpeed=60});
    chassis->waitUntilDone();
    chassis->moveToPoint(47, 6, 1000, {.forwards = true, .maxSpeed = 40});
    chassis->waitUntilDone();

    Task dropGoal([&] {
        delay(300);
        robot->set_lift_intake(false);
        delay(300);
        robot->set_mogo(false);
        delay(100);
        intake->color_sort = false;
        delay(300);
        arm->set_target(ALLIANCE_STAKE - 10);
        intake->forwards(90);
        delay(500);
        intake->set_stop_condition(stage_1_stop);
    });
    // chassis->moveToPoint(20, -47, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed=80});
    chassis->moveToPoint(47, -20, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed=80, .earlyExitRange=5});
    chassis->waitUntilDone();
    chassis->moveToPoint(23, -47, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed=80});
    chassis->waitUntilDone();

    delay(500);

    chassis->turnToPoint(20.5, -21, 500, {.forwards = false});
    chassis->waitUntilDone();

    intake->set_stop_condition(nullptr);
    intake->backwards(20);

    chassis->moveToPoint(20.5, -21, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed=80});
    chassis->waitUntilDone();

    bool clamped = false;
    Task clamptask2([&] {
        while (true) {
            lemlib::Pose pose = chassis->getPose();
            int distanceFrom00 = sqrt(pow(pose.x - 20.5, 2) + pow(pose.y + 21, 2));
            if (distanceFrom00 < 5) {
                robot->set_mogo(true);
                clamped = true;
                chassis->cancelMotion();
                break;
            } 
            delay(10);
        }
    });

    chassis->waitUntilDone();

    if (!clamped) {
        robot->set_mogo(true);
    }

    intake->color_sort = true;
    intake->forwards();
    // delay(750);
    chassis->turnToPoint(12, -12, 500, {.forwards = true, .maxSpeed = 60, .earlyExitRange = 5});
    chassis->waitUntilDone();
    arm->set_target(ALLIANCE_STAKE + 20);
    robot->timedMove(20, 200);

    runningAuton = false;
}