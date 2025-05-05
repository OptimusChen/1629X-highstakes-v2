#include "autonomous.hpp"
#include "lib/util.hpp"
#include "lib/controller/pid.hpp"
#include "paths.hpp"
#include "controls.hpp"
#include "intake.hpp"
#include "arm.hpp"
#include "lib/logging.hpp"
#include "s.hpp"

/*
██████╗░███████╗██████╗░  ███╗░░██╗███████╗░██████╗░░█████╗░████████╗██╗██╗░░░██╗███████╗
██╔══██╗██╔════╝██╔══██╗  ████╗░██║██╔════╝██╔════╝░██╔══██╗╚══██╔══╝██║██║░░░██║██╔════╝
██████╔╝█████╗░░██║░░██║  ██╔██╗██║█████╗░░██║░░██╗░███████║░░░██║░░░██║╚██╗░██╔╝█████╗░░
██╔══██╗██╔══╝░░██║░░██║  ██║╚████║██╔══╝░░██║░░╚██╗██╔══██║░░░██║░░░██║░╚████╔╝░██╔══╝░░
██║░░██║███████╗██████╔╝  ██║░╚███║███████╗╚██████╔╝██║░░██║░░░██║░░░██║░░╚██╔╝░░███████╗
╚═╝░░╚═╝╚══════╝╚═════╝░  ╚═╝░░╚══╝╚══════╝░╚═════╝░╚═╝░░╚═╝░░░╚═╝░░░╚═╝░░░╚═╝░░░╚══════╝
*/
void redNegStart(lemlib::Chassis* chassis, Robot* robot, Arm* arm, Intake* intake) {
    chassis->setPose(-59, 12, 214.5);

    arm->set_target(ALLIANCE_STAKE + 80);

    delay(500);

    chassis->moveToPoint(-17, 23, 3000, {.forwards = false, .maxSpeed = 100});
    intake->forwards();

    bool clamped = false;
    bool bruh = true;
    Task clamptask([&] {
        while (true) {
            lemlib::Pose pose = chassis->getPose();
            int distanceFrom00 = sqrt(pow(pose.x + 17, 2) + pow(pose.y - 23, 2));
            if (distanceFrom00 < 5) {
                robot->set_mogo(true);
                clamped = true;
                delay(200);
                if (bruh) chassis->cancelMotion();
            } 
            delay(10);
        }
    });

    chassis->waitUntilDone();
    bruh = false;

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
    chassis->moveToPoint(-54, 54, 2000, {.forwards = true, .maxSpeed = 127});
    chassis->waitUntilDone();
    chassis->turnToPoint(-70, 70, 200);
    chassis->waitUntilDone();
}
void red6p1Start(lemlib::Chassis* chassis, Robot* robot, Arm* arm, Intake* intake) {
    redNegStart(chassis, robot, arm, intake);
 
    intake->color_sort = false;
    intake->antijam = false;    
    chassis->moveToPoint(-70, 70, 500, {.forwards = true, .maxSpeed = 127, .minSpeed=40});  
    chassis->waitUntilDone();   
    
    chassis->moveToPoint(-47, 47, 1000, {.forwards = false, .maxSpeed = 40});
    chassis->waitUntilDone(); 

    intake->color_sort = true;

    chassis->moveToPoint(-60, 60, 500, {.forwards = true, .maxSpeed = 127, .minSpeed=50});  
    chassis->waitUntilDone(); 
    delay(500);
    
    chassis->moveToPoint(-47, 47, 1000, {.forwards = false, .maxSpeed = 40});
    chassis->waitUntilDone(); 
    intake->color_sort = true;
    intake->antijam = true; 
    
    robot->set_rush_arm_left(true);
    chassis->turnToPoint(-47, 18, 500);
    chassis->waitUntilDone();
    robot->set_rush_arm_left(false);
    Task pistakeTask([&] {
        pros::delay(500);
        robot->set_lift_intake(true);
    });
    chassis->moveToPose(-47, 12, 180, 1000, {.forwards = true, .lead=0.2, .maxSpeed=127, .minSpeed=60});
    chassis->waitUntilDone();
    chassis->moveToPoint(-47, 6, 1000, {.forwards = true, .maxSpeed = 60});
    chassis->waitUntilDone();
    robot->set_lift_intake(false);
}
void worldsautonomous::red6p1CornerClear(lemlib::Chassis* chassis, Robot* robot) {
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    Arm * arm = robot->get_subsystem<Arm>();
    Intake * intake = robot->get_subsystem<Intake>();
    intake->arm = arm; 
    
    intake->color_sort = true;
    intake->set_color(RED);  

    auto stage_1_stop = [&] {
        auto opticalMeasure = intake->optical->get_hue();
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

    red6p1Start(chassis, robot, arm, intake);

    robot->set_rush_arm_right(true);
    chassis->moveToPoint(-62, -56, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed=80});
    chassis->waitUntilDone();

    chassis->turnToPoint(0, 0, 1000);
    chassis->waitUntilDone();

    chassis->moveToPoint(-62, -62, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed=127});
    chassis->waitUntilDone();

    runningAuton = false;
}
void worldsautonomous::red6p1CornerNoSweep(lemlib::Chassis* chassis, Robot* robot) {
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    Arm * arm = robot->get_subsystem<Arm>();
    Intake * intake = robot->get_subsystem<Intake>();
    intake->arm = arm; 
    
    intake->color_sort = true;
    intake->set_color(RED);  

    auto stage_1_stop = [&] {
        auto opticalMeasure = intake->optical->get_hue();
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

    red6p1Start(chassis, robot, arm, intake);

    chassis->moveToPoint(-62, -30, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed=80});
    chassis->waitUntilDone();

    chassis->turnToPoint(0, 0, 1000);
    chassis->waitUntilDone();

    runningAuton = false;
}
void worldsautonomous::red6p1Ladder(lemlib::Chassis* chassis, Robot* robot) {
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    Arm * arm = robot->get_subsystem<Arm>();
    Intake * intake = robot->get_subsystem<Intake>();
    intake->arm = arm; 
    
    intake->color_sort = true;
    intake->set_color(RED);  

    auto stage_1_stop = [&] {
        auto opticalMeasure = intake->optical->get_hue();
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

    red6p1Start(chassis, robot, arm, intake);

    delay(200);
    chassis->moveToPoint(-47, 0, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed=60});
    chassis->waitUntilDone();
    
    chassis->turnToPoint(0, 0, 500);
    chassis->waitUntilDone();

    chassis->moveToPoint(-32, 0, 1000, {.forwards = true, .maxSpeed = 60});
    chassis->waitUntilDone();

    arm->set_target(ALLIANCE_STAKE + 40);

    delay(1200);

    runningAuton = false;
}

/*
██████╗░██╗░░░░░██╗░░░██╗███████╗  ██████╗░░█████╗░░██████╗██╗████████╗██╗██╗░░░██╗███████╗
██╔══██╗██║░░░░░██║░░░██║██╔════╝  ██╔══██╗██╔══██╗██╔════╝██║╚══██╔══╝██║██║░░░██║██╔════╝
██████╦╝██║░░░░░██║░░░██║█████╗░░  ██████╔╝██║░░██║╚█████╗░██║░░░██║░░░██║╚██╗░██╔╝█████╗░░
██╔══██╗██║░░░░░██║░░░██║██╔══╝░░  ██╔═══╝░██║░░██║░╚═══██╗██║░░░██║░░░██║░╚████╔╝░██╔══╝░░
██████╦╝███████╗╚██████╔╝███████╗  ██║░░░░░╚█████╔╝██████╔╝██║░░░██║░░░██║░░╚██╔╝░░███████╗
╚═════╝░╚══════╝░╚═════╝░╚══════╝  ╚═╝░░░░░░╚════╝░╚═════╝░╚═╝░░░╚═╝░░░╚═╝░░░╚═╝░░░╚══════╝
*/

void worldsautonomous::bluePos6Ladder(lemlib::Chassis* chassis, Robot* robot) {
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    Arm * arm = robot->get_subsystem<Arm>();
    Intake * intake = robot->get_subsystem<Intake>();
    intake->arm = arm; 
    
    intake->color_sort = true;
    intake->set_color(BLUE);  

    auto stage_1_stop = [&] {
        auto opticalMeasure = intake->optical->get_hue();
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

    chassis->setPose(51.5, -23, 90);

    chassis->moveToPoint(17, -24, 1900, {.forwards = false, .maxSpeed = 100});

    bool clamped = false;
    Task clampTask([&] {
        while (true) {
            lemlib::Pose pose = chassis->getPose();
            int distanceFrom00 = sqrt(pow(pose.x - 17, 2) + pow(pose.y + 24, 2));
            if (distanceFrom00 < 5) {
                robot->set_mogo(true);
                clamped = true;
                delay(200);
                chassis->cancelMotion();
                break;
            } 
            delay(10);
        }
    });

    chassis->waitUntilDone();

    if (!clamped) {
        robot->set_mogo(true);
        delay(200);
    }

    intake->color_sort = false;
    float pos = intake->hooks.get_position();
    intake->forwards();
    while (intake->hooks.get_position() < pos + 900) {
        delay(10);
    }

    intake->stop();
    intake->color_sort = true;

    chassis->turnToPoint(0, 8, 700);
    chassis->waitUntilDone();
    chassis->moveToPose(9, -8, 335, 1000, {.forwards = true, .lead=0.1});
    chassis->waitUntilDone();

    intake->stop();
    robot->set_rush_arm_left(true);

    delay(400);

    chassis->turnToPoint(23, -12, 500, {.forwards = false});
    chassis->waitUntilDone();
    intake->backwards(20);
    chassis->moveToPoint(23, -12, 3000, {.forwards = false, .maxSpeed = 80, .minSpeed=60});
    chassis->waitUntilDone();

    chassis->turnToPoint(18, -45, 1000, {.forwards = true, .maxSpeed = 40});
    chassis->waitUntilDone();

    robot->set_rush_arm_left(false);
    delay(500);

    chassis->turnToPoint(23, -45, 500, {.forwards = true, .maxSpeed = 40});
    chassis->waitUntilDone();

    robot->set_rush_arm_left(true);
    robot->set_rush_arm_right(true);

    intake->forwards();
    chassis->moveToPoint(23, -48, 2000, {.forwards = true, .maxSpeed = 60, .minSpeed=40});
    chassis->waitUntilDone();
    robot->set_rush_arm_left(false);
    robot->set_rush_arm_right(false);

    chassis->swingToPoint(54, -54, DriveSide::RIGHT, 500, {.forwards = true, .maxSpeed = 127});
    chassis->waitUntilDone();
    chassis->moveToPoint(54, -54, 2000, {.forwards = true, .maxSpeed = 127});
    chassis->waitUntilDone();

    chassis->turnToPoint(70, -70, 200);
    chassis->waitUntilDone();

    intake->antijam = false;    
    chassis->moveToPoint(70, -70, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed=40});  
    chassis->waitUntilDone();   
    
    chassis->moveToPoint(49, -49, 1000, {.forwards = false, .maxSpeed = 40});
    chassis->waitUntilDone(); 

    intake->color_sort = true;

    chassis->moveToPoint(60, -60, 500, {.forwards = true, .maxSpeed = 127, .minSpeed=50});  
    chassis->waitUntilDone(); 
    delay(500);
    
    chassis->moveToPoint(47, -47, 1000, {.forwards = false, .maxSpeed = 40});
    chassis->waitUntilDone(); 
    intake->color_sort = true;
    intake->antijam = true; 

    delay(10000);





    pros::delay(490);
    robot->set_rush_arm_right(true);
    chassis->swingToPoint(-24, 1, DriveSide::LEFT, 800, {.minSpeed = 75});
    chassis->waitUntilDone();
    pros::delay(320);
    robot->set_rush_arm_left(true);
    chassis->swingToPoint(0, 24, DriveSide::LEFT, 500, {.minSpeed = 75});
    chassis->waitUntilDone();
    intake->backwards(60);
    chassis->moveToPoint(39, -32, 1200, {.forwards = false, .minSpeed = 60});
    chassis->waitUntilDone();
    chassis->turnToPoint(24, -30, 900, {.maxSpeed = 60});
    chassis->waitUntilDone();
    pros::delay(500);
    
    robot->set_rush_arm_left(false);
    robot->set_rush_arm_right(false);

    runningAuton = false;
}

void worldsautonomous::red_sawp(lemlib::Chassis* chassis, Robot* robot) {
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    Arm * arm = robot->get_subsystem<Arm>();
    Intake * intake = robot->get_subsystem<Intake>();
    intake->arm = arm; 
    
    intake->color_sort = true;
    intake->set_color(RED);

    auto stage_1_stop = [&] {
        auto opticalMeasure = intake->optical->get_hue();
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
    chassis->moveToPoint(-70, 70, 500, {.forwards = true, .maxSpeed = 127, .minSpeed=60});  
    chassis->waitUntilDone();   
    
    chassis->moveToPoint(-47, 47, 1000, {.forwards = false, .maxSpeed = 127});
    chassis->waitUntilDone();
    intake->color_sort = true;
    intake->antijam = true; 
    
    robot->set_rush_arm_left(true);
    chassis->turnToPoint(-47, 18, 500);
    chassis->waitUntilDone();
    robot->set_rush_arm_left(false);
    chassis->moveToPose(-47, 12, 180, 1000, {.forwards = true, .lead=0.2, .maxSpeed=127, .minSpeed=60});

    Task intakeLiftDelay([&] {
        delay(800);
        robot->set_lift_intake(true);
    });

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
        intake->forwards();
        intake->set_stop_condition(stage_1_stop);
        delay(500);
    });
    chassis->moveToPoint(-20, -47, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed=80});
    chassis->waitUntilDone();

    delay(500);

    chassis->turnToPoint(-20.5, -21, 500, {.forwards = false});
    chassis->waitUntilDone();

    intake->set_stop_condition(nullptr);

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
    chassis->setPose(56, 12, 145.5);

    arm->set_target(ALLIANCE_STAKE + 80);

    delay(500);

    chassis->moveToPoint(17, 23, 3000, {.forwards = false, .maxSpeed = 100});
    intake->forwards();

    bool clamped = false;
    Task clamptask([&] {
        while (true) {
            lemlib::Pose pose = chassis->getPose();
            int distanceFrom00 = sqrt(pow(pose.x - 17, 2) + pow(pose.y - 23, 2));
            if (distanceFrom00 < 9) {
                robot->set_mogo(true);
                clamped = true;
                delay(200);
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

    auto stage_1_stop = [&] {
        auto opticalMeasure = intake->optical->get_hue();
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
    robot->set_lift_intake(false);

    // chassis->moveToPoint(20, -47, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed=80});
    chassis->moveToPoint(47, -20, 1000, {.forwards = true, .maxSpeed = 60, .minSpeed=0, .earlyExitRange=5});
    chassis->waitUntilDone();

    Task ihatethis([&] {
        delay(500);
        robot->set_mogo(false);
        delay(100);
        intake->color_sort = false;
    });
    Task dropGoal([&] {
        delay(100);
        arm->set_target(ALLIANCE_STAKE - 10);
        intake->forwards();
        delay(500);
        intake->set_stop_condition(stage_1_stop);
    });
    chassis->moveToPoint(23, -47, 1000, {.forwards = true, .maxSpeed = 127, .minSpeed=80});
    chassis->waitUntilDone();

    delay(500);

    chassis->turnToPoint(23.5, -21, 500, {.forwards = false});
    chassis->waitUntilDone();

    intake->set_stop_condition(nullptr);

    chassis->moveToPoint(23.5, -21, 2000, {.forwards = false, .maxSpeed = 100});
    chassis->waitUntilDone();

    bool clamped = false;
    Task clamptask2([&] {
        while (true) {
            lemlib::Pose pose = chassis->getPose();
            int distanceFrom00 = sqrt(pow(pose.x - 21.5, 2) + pow(pose.y + 21, 2));
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