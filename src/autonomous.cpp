#include "autonomous.hpp"
#include "lib/util.hpp"
#include "lib/controller/pid.hpp"
#include "paths.hpp"
#include "controls.hpp"
#include "intake.hpp"
#include "arm.hpp"

MPConstraint fast{60, 150, INCH};
MPConstraint medium{50, 150, INCH};
MPConstraint huh{60, 100, INCH};
MPConstraint huh2{55, 150, INCH};
MPConstraint huh3{50, 150, INCH};
MPConstraint slow{40, 150, INCH};
MPConstraint kinda_slow{45, 150, INCH};

#define SPAM 2

/*

SKILLS

*/

void bestautonfr::casey(Robot* robot, lemlib::Chassis* chassis) {
    robot->set_pose(-61, 0, 0);
    robot->poseSet = true;
    robot->calibrate();

    // robot->turnToHeading(90, 500);
    // return;

    robot->initialize_particle_filter();
    robot->set_pose_mode(MCL);

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();
    Distance front(F_DISTANCE);

    intake->arm = arm;
    intake->color_sort = false;
    intake->antijam = true;

    Task updates([&]() {
        while (true) {
            arm->update();
            intake->update();
            delay(1);
        }
    });

    auto stage_1_stop = [&] {
        return intake->detected_ring(STAGE_1);
    };
    auto stage_1_stop_color = [&] {
        auto hue = intake->optical.get_hue();
        return intake->detected_ring(STAGE_1) && (hue > 0 && hue < 30);
    };
    auto stage_2_stop = [&] {
        return intake->detected_ring(STAGE_2);
    };

    intake->forwards();
    pros::delay(200);
    intake->stop();

    //-----------CLAMP FIRST MOGO------------------//

    robot->moveToPoint(-45, 0, 1000, true, false, 100);

    robot->turnToHeading(270, 500);

    robot->moveToPoint(-45, 24, 1000, false, false, 80);

    robot->set_mogo(true);

    //-----------SCORE ONE RING------------------//

    robot->turnToHeading(0, 500);

    intake->forwards();

    robot->moveToPoint(-24, 20, 750, true, false, 100);

    robot->turnToHeading(41, 500);

    //-----------OBTAIN 2nd RING------------------//

    robot->moveToPoint(0, 43, 1150, true, false, 100);

    robot->turnToHeading(8, 500);

    arm->set_target(LOAD);

    robot->moveToPoint(24, 48, 850, true, false, 100);
    robot->timedMove(20, 400);

    robot->moveToPoint(0, 42, 1000, false, false, 80);

    for (int i = 0; i < 3; i++) {
        intake->stop();
        delay(100);
        intake->forwards();
        delay(100);
    }
    
    intake->stop();
    delay(200);
    arm->set_target(MID + 20);
    pros::delay(500);
    intake->forwards();

    //-----------LADY BROWN 1------------------//

    // robot->turnToPoint(0, 68, 500);
    
    robot->turnToHeading(90, 500);
    robot->moveToPoint(0, 68, 850, true, true, 80);

    arm->set_target(SCORE);
    robot->timedMove(40, 200);
    pros::delay(450);

    //-----------SCORE THREE RINGS------------------//

    robot->moveToPoint(0, 46, 750, false, false, 100);
    arm->set_target(REST);

    // robot->turnToPoint(-33, 46, 500);

    robot->moveToPoint(-33, 46, 500+1000, true, true, 90);

    robot->moveToPoint(-60, 46, 2000, true, false, 60);

    pros::delay(250);

    // robot->turnToPoint(-49, 57, 500);

    robot->moveToPoint(-49, 57, 500+1000, true, true, 80);
    robot->timedMove(40, 200);

    //-----------Place in corner------------------//

    // robot->turnToPoint(-60, 60, 500, {.forwards = false});
    // chassis.waitUntilDone();

    robot->moveToPoint(-60, 60, 500+500, false, true, 100);
    // chassis.waitUntilDone();

    robot->set_mogo(false);
    intake->stop();

    robot->moveToPoint(-48, 48, 750, true, false, 100);
    arm->set_target(LOAD);

    ///////////////////////////////////////////////
    //--------------2ND SECTION----------------//
    ///////////////////////////////////////////////


    //Get ring for alliance stake

    robot->moveToPoint(36, 48, 1500, true, false, 100);
    intake->forwards();

    robot->moveToPoint(47, 48, 1000, true, false, 60);
    
    //Clamp second goal

    robot->moveToPoint(62, 22, 1500, false, false, 90);
    // robot->timedMove(40, 500);
    robot->set_mogo(true);

    //put in corner
    robot->turnToHeading(90, 500);
    robot->set_rush_arm_right(true);

    robot->moveToPoint(64, 62, 1500, true, true, 100);

    robot->turnToHeading(180, 1500, false, 0, 60);
    robot->set_rush_arm_right(false);
    pros::delay(100);

    robot->moveToPoint(64, 64, 1000, false, true, 80);
    robot->set_mogo(false);
    arm->set_target(MID);
    pros::delay(500);
    intake->forwards();

    //get third goal
    robot->moveToPoint(48, 40, 1250, true, true, 100);
    
    robot->turnToHeading(90, 500);
    intake->stop();
    delay(500);
    intake->backwards();
    
    Task mogodelay1([&] {
        pros::delay(1350);
        robot->set_mogo(true);
    });  
    robot->moveToPoint(48, 0, 1750, false, false, 80);

    //score alliance stake

    int target = 1750;
    float error = front.get_distance() - target;
    while (abs(error) > 30) {
        robot->left->move(15 * util::sign(error));
        robot->right->move(15 * util::sign(error));
        error = front.get_distance() - target;

        delay(5);
    }
    robot->left->move(0);
    robot->right->move(0);

    robot->turnToHeading(0, 750);
    robot->moveToPoint(68, 0, 1000, true, false, 127);
    // robot->timedMove(80, 300);
    
    robot->moveToPoint(52, 0, 1000, false, false, 60, true);

    arm->set_target(ALLIANCE_STAKE + 30);
    pros::delay(500); 

    arm->motor->set_brake_mode(E_MOTOR_BRAKE_COAST);

    robot->moveToPoint(44, 0, 2000, false);
    intake->stop();
    
    ///////////////////////////////////////////////
    //--------------3RD SECTION----------------//
    ///////////////////////////////////////////////

    //score one ring
    arm->motor->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    arm->set_target(REST);
    delay(750);
    intake->forwards();

    robot->moveToPoint(24, 24, 500+1000, true, true, 100);

    //cross under ladder score 3
    robot->turnToHeading(225, 1000);
    intake->stop();

    robot->moveToPoint(24, 24, 500, true, false, 40);

    robot->set_pose_mode(ODOM);
    robot->set_pose(24, 24, robot->get_pose().get_degrees());

    // robot->moveToPoint(-24, -24, 2500, true, false, 80, true);
    robot->relative(18, 80, 2500);
    intake->forwards();
    robot->timedMove(20, 500);
    robot->set_pose_mode(MCL);

    delay(1000);

    robot->moveToPoint(-44, -44, 1000, true, false, 80);
    
    //score two and corner
    robot->moveToPoint(-62, -48, 2000, true, true, 60);

    pros::delay(250);

    robot->moveToPoint(-44, -62, 2000, true, true, 80);
    robot->timedMove(40, 300);

    //-----------Place in corner------------------//

    robot->moveToPoint(-64, -64, 750, false, true, 100);

    intake->stop();

    // linearController.kD = 3;
    // angularController.kD = 19;n
    
    robot->turnToPoint(-48, -48, 500);
    robot->set_mogo(false);

    robot->moveToPoint(-48, -48, 750, true, false, 100);

    ///////////////////////////////////////////////
    //--------------4th SECTION----------------//
    ///////////////////////////////////////////////
    
    robot->moveToPoint(-48, -24, 500+1250, false, true, 80);
    robot->set_mogo(true);
    
    robot->turnToPoint(-24, -48, 500);
    intake->forwards();
    arm->set_target(LOAD);

    robot->moveToPoint(-24, -48, 1000, true, false, 80);

    //lady brown stuff

    robot->moveToPoint(4, -40, 1500, true, true, 80);
    for (int i = 0; i < 3; i++) {
        intake->stop();
        delay(100);
        intake->forwards();
        delay(100);
    }
    arm->set_target(MID + 20);
    intake->stop();
    pros::delay(500);

    //-----------LADY BROWN 1------------------//
    
    robot->turnToHeading(270, 500);
    robot->moveToPoint(2, -68, 1000, true, false, 80);

    arm->set_target(SCORE + 50);
    pros::delay(500);

    delay(100000);

    // //now score rings

    // chassis.moveToPoint(0, -42, 750, {.forwards=false, .maxSpeed = 100});
    // chassis.waitUntilDone();
    // lift.setState(lib::LiftState::Stored);

    // intake.setState(lib::IntakeState::In);

    // chassis.turnToPoint(24, -48, 500);
    // chassis.waitUntilDone();

    // chassis.moveToPoint(24, -48, 850, {.forwards=true, .maxSpeed =100});
    // chassis.waitUntilDone();

    // chassis.turnToPoint(24, -24, 500);
    // chassis.waitUntilDone();

    // chassis.moveToPoint(24, -24, 850, {.forwards=true, .maxSpeed =100});
    // chassis.waitUntilDone();

    // pros::delay(350);

    // chassis.turnToPoint(44, -44, 500);
    // chassis.waitUntilDone();

    // chassis.moveToPoint(44, -44, 1000, {.forwards=true, .maxSpeed =90});
    // intake.setState(lib::IntakeState::In);
    // chassis.waitUntilDone();

    // //score two and corner
    // chassis.turnToPoint(48, -60, 500);
    // chassis.waitUntilDone();

    // chassis.moveToPoint(48, -60, 500, {.forwards=true, .maxSpeed =80});
    // chassis.waitUntilDone();

    // pros::delay(250);

    // chassis.turnToPoint(60, -44, 500);
    // chassis.waitUntilDone();

    // chassis.moveToPoint(60, -44, 500, {.forwards=true, .maxSpeed =80});

    // chassis.waitUntilDone();

    // //put in corner

    // chassis.turnToPoint(62, -62, 500, {.forwards = false});
    // chassis.waitUntilDone();

    // chassis.moveToPoint(62, -62, 1000, {.forwards=false, .maxSpeed =127});
    // pros::delay(850);
    // clamp.set_value(false);
    // chassis.waitUntilDone();


    // intake.setState(lib::IntakeState::Idle);

    // //HANGGG
    // chassis.moveToPoint(24, -24, 500, {.forwards=true, .maxSpeed =127});
    // lift.setState(lib::LiftState::Mid);
    // chassis.waitUntilDone();

    // chassis.turnToPoint(64, -64, 500);
    // chassis.waitUntilDone();

    // chassis.moveToPoint(8, -8, 1000, {.forwards=false, .maxSpeed =127});
    // chassis.waitUntilDone();
}

void bestautonfr::skills(Robot* robot) {
    // robot->set_pose(0, 0, 90);
    // robot->poseSet = true;
    // robot->calibrate();
    // robot->set_pose_mode(ODOM);  

    // robot->ramsete({{0, 0}, {0, 30}, {30, 0}, {30, 30}}, fast);
    // return;

    robot->set_pose(-58, 0, 0);
    robot->poseSet = true;
    robot->calibrate();

    robot->initialize_particle_filter();
    robot->set_pose_mode(MCL);  

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();
    Distance front(F_DISTANCE);

    intake->arm = arm;
    intake->color_sort = false;
    intake->antijam = true;

    Task updates([&]() {
        while (true) {
            arm->update();
            intake->update();
            delay(1);
        }
    });

    auto stage_1_stop = [&] {
        return intake->detected_ring(STAGE_1);
    };
    auto stage_1_stop_color = [&] {
        auto hue = intake->optical.get_hue();
        return intake->detected_ring(STAGE_1) && (hue > 0 && hue < 30);
    };
    auto stage_2_stop = [&] {
        return intake->detected_ring(STAGE_2);
    };

    intake->forwards();
    delay(500);

    delay(250);

    // CORNER 1

    robot->moveToPoint(-47.5, robot->get_pose().y, 1000, true, false, 80);
    robot->turnToHeading(90, 750);
    robot->moveToPoint(robot->get_pose().x, -22, 500, false, false, 80);
    robot->moveToPoint(robot->get_pose().x, -25.7, 1000, false, false, 30);

    arm->set_target(REST);

    robot->set_mogo(true);

    // delay(1000000);

    // return;

    intake->forwards();

    // robot->turnToHeading(35, 500);

    robot->ramsete({
        {-47.0, -25.70}, {-48.10, -9.80}, {14.55, -38.70}, {11.60, -51.21}
    }, huh);

    robot->ramsete({
        {11.60, -51.21}, {8.06, -66.20}, {-4.84, -40.11}, {-50.33, -47.50}, 
        {-50.33, -47.50}, {-71.10, -50.90}, {-58.00, -64.20}, {-30.00, -58.60}
    }, huh3);

    intake->antijam = false;
    robot->timedMove(-50, 1000);

    robot->set_mogo(false);
    intake->antijam = true;

    // WALL STAKE

    intake->forwards();
    Task abc6([&] {
        delay(2000);
        arm->set_target(LOAD);
    });
    
    robot->ramsete({{-60.00, -60.00}, {3.64, -58.30}, {19.95, -40.11}, {29.21, -51.40}}, fast);
    robot->timedMove(20, 500);
    robot->moveToPoint(25.91, -49.88, 500, false);
    robot->turnToHeading(315, 500);

    for (int i = 0; i < SPAM; i++) {
        intake->stop();
        delay(100);
        intake->forwards();
        delay(100);
    }
    intake->stop();

    intake->forwards();

    Task abc3([&] {
        delay(200);
        intake->stop();
        arm->set_target(MID);
        delay(250);
        intake->forwards(80);
    });
    
    intake->set_stop_condition(stage_1_stop_color);
    robot->ramsete({{25.91, -49.88}, {32.92, -57.18}, {37.43, -60.72}, {40.61, -60.72}}, slow);
    robot->timedMove(20, 800);
    robot->moveToPoint(40, -60, 1000, false, false, 40);

    intake->backwards(20);
    robot->ramsete({{40.00, -59.72}, {17.32, -59.90}, {-0.03, -61.40}, {0.00, -52.75}}, fast , BACKWARDS);

    robot->moveToPoint(1, -61, 750, true, false, 60);

    // start logic ws
    robot->timedMove(40, 400);
    arm->set_target(SCORE + 50);
    delay(750);
    arm->set_target(LOAD);
    // robot->timedMove(-40, 250);
    delay(750);

    intake->set_stop_condition(nullptr);
    intake->forwards();
    delay(500);
    for (int i = 0; i < 1; i++) {
        intake->stop();
        delay(100);
        intake->forwards();
        delay(100);
    }
    intake->stop();
    robot->timedMove(20, 400);
    arm->set_target(SCORE + 50);
    robot->timedMove(20, 500);
    arm->set_target(LOAD);
    robot->moveToPoint(0, -48, 500, false, false, 60);
    // end logic ws

    // CORNER 2
    robot->ramsete({{0, -54}, {1.00, -35.67}, {47.14, -32.73}, {59.17, -23.35}}, medium, BACKWARDS);
    robot->timedMove(-30, 200);
    robot->set_mogo(true);  

    intake->forwards(); 
    // robot->ramsete({{58.32, -24.02}, {23.69, -24.02}, {3.70, -48.00}, {31.57, -48.00}}, fast);
    arm->set_target(LOAD);
    robot->turnToHeading(235, 500);
    robot->moveToPoint(47.5, -42, 1000, true, true, 70);
    // robot->timedMove(20, 500);
    delay(1000);

    Task abc2([&] {
        for (int i = 0; i < SPAM; i++) {
            intake->stop();
            delay(100);
            intake->forwards();
            delay(100);
        }
        intake->stop();
        arm->set_target(MID);
    });
    robot->moveToPoint(53, -30, 500, false, false, 80);

    intake->forwards();
    intake->set_stop_condition(stage_1_stop_color);
    robot->moveToPoint(58, -45, 1000, true, false, 60);

    robot->turnToHeading(120, 750);
    robot->set_mogo(false);
    robot->timedMove(-60, 500);
    // robot->moveToPoint(58, -59, 1000, false, false, 60);

    arm->set_target(LOAD);

    // MIDDLE
    robot->ramsete({{58.00, -59.00}, {58.00, -45.47}, {47.23, -35.86}, {47.23, -18.80}}, medium);
    robot->turnToHeading(270, 750);
    Task abc([&] {
        delay(1500);
        robot->set_mogo(true);
    });
    robot->moveToPoint(47, 10, 2000, false, false, 50);

    robot->moveToPoint(47, 0, 1000, true, false, 80);
    robot->turnToHeading(0, 500);
    // robot.move

    // robot->moveToPoint(62, robot->get_pose().y, 1000, true, false, 50, true);

    robot->ramsete({{47.00, 12.00}, {47.00, 3.78}, {52.81, 0.00}, {61.81, 0.00}}, medium);
    robot->timedMove(20, 500);

    // const int target = 310;
    // float error = front.get_distance() - target;
    // while (abs(error) > 30) {
    //     robot->left->move(20 * util::sign(error));
    //     robot->right->move(20 * util::sign(error));
    //     error = front.get_distance() - target;

    //     delay(5);
    // }
    // robot->left->move(0);
    // robot->right->move(0);
    // robot->moveToPoint(62, robot->get_pose().y, 1000, true, false, 80);
    robot->moveToPoint(52, robot->get_pose().y, 1000, false, false, 50, true);

    arm->set_target(ALLIANCE_STAKE + 20);
    delay(500);

    intake->set_stop_condition(nullptr);
    intake->color_sort = true;
    intake->forwards();
    robot->moveToPoint(47, 0, 1000, false, false);
    // arm->set_target(REST);

    // CORNER 3

    arm->set_target(REST);
    robot->turnToHeading(125, 500);
    intake->color_sort = false;
    
    // intake->color_sort = true;
    robot->ramsete({
        {47.00, 0.00}, {42.57, 9.99}, {26.42, 15.26}, {23.60, 23.40}, 
        {23.60, 23.40}, {20.28, 32.98}, {19.97, 57.16}, {31.44, 57.02}
    }, huh);
    delay(500);

    robot->set_pose(31.532, 60.369, 0);
    robot->set_pose_mode(ODOM);
    intake->color_sort = true;
    robot->moveToPoint(50, 57, 500, true, false);
    robot->turnToHeading(310, 500);
    // robot->moveToPoint(59, 48, 1000, true, false);
    robot->timedMove(60, 800);
    // robot->timedMove(-60, 600);
    robot->moveToPoint(50, 57, 500, false, false);
    robot->turnToHeading(270, 500);
    robot->timedMove(90, 450);
    // return;
    robot->turnToHeading(225, 500);
    robot->set_mogo(false);
    robot->timedMove(-90, 500);
    robot->timedMove(-60, 500);

    robot->set_pose_mode(MCL);

    intake->backwards();
    intake->color_sort = false;
    
    Task abc5([&] {
        delay(500);
        intake->forwards();
        delay(1500);
        arm->set_target(LOAD);
    });
    robot->ramsete({
        {52.44, 52.78}, {35.44, 33.65}, {-0.24, 32.51}, {3.00, 59.00}
    }, medium);
    delay(500);
    for (int i = 0; i < SPAM + 1; i++) {
        intake->stop();
        delay(100);
        intake->forwards();
        delay(100);
    }
    intake->stop();
    robot->timedMove(40, 500);
    arm->set_target(SCORE + 50);
    delay(500);
    robot->moveToPoint(0, 54, 1000, false, false, 60);
    arm->set_target(REST);

    robot->ramsete({{0.00, 54.00}, {-0.00, 31.26}, {-32.81, 40.70}, {-47.28, 23.31}}, medium, BACKWARDS);
    robot->timedMove(-20, 500);
    robot->set_mogo(true);
    robot->turnToHeading(0, 500);
    robot->ramsete({
        {-47.11, 23.48}, {-31.77, 23.48}, {-13.51, 22.79}, {-18.68, 38.47}
    }, {55, 200, INCH});
    robot->ramsete({
        {-18.68, 38.47}, {-20.38, 43.65}, {-24.36, 45.71}, {-48.83, 45.71},
        {-48.83, 45.71}, {-73.30, 45.71}, {-53.31, 62.94}, {-40.73, 57.60}
    }, huh3);
    
    robot->moveToPoint(-60, 60, 1000, false, false);
    robot->set_mogo(false);

    delay(100000000);

    return;

    return;

    robot->ramsete({
        {26.00, 53.00}, {31.63, 49.55}, {36.92, 46.02}, {45.59, 57.39}
    }, huh);

    delay(500);

    robot->moveToPoint(47, 58, 2000, true, false, 60);
    robot->ramsete({{26.00, 53.00}, {31.63, 49.55}, {34.44, 42.76}, {43.10, 54.14}}, huh);

    robot->moveToPoint(60, 45, 1000, true, false, 60);
    robot->turnToHeading(180, 500);
    robot->moveToPoint(60, 60, 1000, false, false, 60);
    return;
    // delay(10000);

    // return;

    // robot->turnToHeading(125, 250);

    // robot->ramsete({
    //     {17.76, -23.13}, {10.52, -12.50}, {2.60, -12.50}, {2.60, 0.00}
    // }, huh);
    // intake->forwards(100);
    // intake->set_stop_condition(nullptr);
    // delay(500);
    // intake->set_stop_condition(stage_1_stop);

    // robot->turnToHeading(45, 500);
    // robot->moveToPoint(23.5, 23.5, 1000, true, false, 100);
    intake->set_stop_condition(nullptr);
    intake->forwards();

    // CORNER 3
    
    
    robot->ramsete({{23.50, 23.50}, {32.35, 30.75}, {45.43, 8.77}, {45.43, 37.90}}, huh2);

    return;
    robot->moveToPoint(45, 48, 1500, true, false, 50);
    robot->moveToPoint(45, 58, 3000, true, false, 50);
    robot->moveToPoint(45, 36, 1000, false, false, 100);

    arm->set_target(LOAD);
    delay(500);
    robot->moveToPoint(55, 44, 1000, true, false, 80);
    delay(1000);
    for (int i = 0; i < SPAM; i++) {
        intake->stop();
        delay(100);
        intake->forwards();
        delay(100);
    }
    intake->stop();
    arm->set_target(MID);

    robot->turnToHeading(240, 500);
    robot->moveToPoint(60, 60, 500, false, false, 80);
    intake->forwards();
    // delay(1000);

    robot->set_mogo(false);

    Task t1([&] {
        intake->backwards();
    });
    robot->ramsete({{62.44, 52.99}, {62.44, 37.68}, {26.24, 47.61}, {-4.17, 47.61}}, medium);

    intake->stop();

    // WALL STAKE 2

    robot->moveToPoint(0, 42, 500, false, false, 80);
    robot->turnToHeading(90, 500);
    intake->set_stop_condition(stage_1_stop);
    intake->forwards(100);
    robot->moveToPoint(0, 61, 500, true, false, 80);

    arm->set_target(SCORE);
    robot->timedMove(40, 500);
    delay(750);
    arm->set_target(LOAD);
    robot->timedMove(-40, 250);
    delay(250);

    intake->set_stop_condition(nullptr);
    intake->forwards();
    robot->timedMove(40, 750);
    for (int i = 0; i < SPAM; i++) {
        intake->stop();
        delay(100);
        intake->forwards();
        delay(100);
    }
    intake->stop();
    arm->set_target(SCORE);
    robot->timedMove(40, 500);
    robot->timedMove(-80, 400);
    arm->set_target(LOAD);
    robot->moveToPoint(0, 54, 500, true, false, 40);
    arm->set_target(REST);
    delay(500);

    // CORNER 4

    robot->turnToHeading(270, 500);
    intake->set_stop_condition(stage_1_stop);
    intake->forwards(80);
    robot->ramsete({
        {0.00, 54.00}, {0.00, 35.50}, {-17.41, 38.32}, {-30.41, 32.02}, 
        {-30.41, 32.02}, {-43.41, 25.72}, {-33.39, 30.85}, {-40.84, 27.12}
    }, medium);
    // robot->turnToHeading(0, 500);
    // robot->moveToPoint(-50, 24, 3000, false, false, 50);
    robot->timedMove(-20, 250);
    robot->set_mogo(true);

    robot->moveToPoint(-50, 24, 3000, false, false, 50);
    robot->turnToHeading(0, 500);

    intake->set_stop_condition(nullptr);
    intake->forwards();

    robot->ramsete({
        {-50.56, 23.33}, {-28.09, 23.33}, {-22.37, 19.89}, {-13.69, 32.51}, 
        {-13.69, 32.51}, {-5.00, 45.13}, {8.65, 37.01}, {3.95, 45.60},
        {3.95, 45.60}, {-5.00, 61.92}, {-22.37, 42.97}, {-53.08, 46.95},
        {-53.08, 46.95}, {-67.00, 48.76}, {-48.51, 62.29}, {-39.56, 58.37}
    }, huh);

    robot->moveToPoint(-60, 60, 1000, false, false, 60);
    robot->set_mogo(false);

    intake->stop();

    return;

    // HANG

    arm->set_target(SCORE + 30);
    robot->ramsete({{-60.00, 60.00}, {-26.92, 60.00}, {-27.34, 30.03}, {-18.03, 18.24}}, medium);
    robot->turnToHeading(145, 500);
    robot->timedMove(-127, 1000);
    robot->timedMove(127, 1000);

    return;
}

float ret(Rotation rot) {
    float measure = rot.get_angle() / 100.0f;
    if (measure > 355) return 0;
    return measure;
}

void init(Robot* robot) {
    robot->initialize_particle_filter();
    robot->set_pose_mode(MCL);  

    delay(1000);

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();

    Task updates([&]() {
        while (true) {
            arm->update();
            intake->update();
            delay(1);
        }
    });
}

void bestautonfr::rush(Robot* robot) {
    robot->set_pose_mode(ODOM);

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();
    intake->arm = arm;
    intake->color_sort = true;

    Task updates([&]() {
        while (true) {
            arm->update();
            intake->update();
            delay(1);
        }
    });

    auto stage_1_stop = [&] {
        return intake->detected_ring(STAGE_2);
    };

    robot->set_pose(-51.001, 29.141, 22);

    robot->set_rush_arm_left(true);
    intake->antijam = false;
    intake->forwards(100);
    intake->set_stop_condition(stage_1_stop);
    robot->moveToPoint(-9, 43, 1000, true, false, 127, true);

    robot->moveToPoint(-35.4, 34.1, 1200, false, false, 60);
    robot->set_rush_arm_left(false);
    intake->antijam = true;
    robot->turnToHeading(0, 500);
    robot->set_pose(-35, 35, 0);
    robot->timedMove(-20, 500);
    // return;

    robot->turnToHeading(120, 500);
    robot->moveToPoint(-23.5, 23.5, 1000, false, false, 60);
    robot->timedMove(-40, 500);
    robot->set_mogo(true);

    intake->set_stop_condition(nullptr);
    intake->forwards();

    robot->turnToHeading(90, 500);
    robot->moveToPoint(-25.5, 55, 1000, true, false, 80);

    robot->timedMove(40, 500);
    delay(500);

    robot->moveToPoint(-37, 13, 1500, false, false, 80);
    robot->moveToPoint(-50, 13, 1000, true, false, 60);
    // delay(500);
    robot->timedMove(-60, 500);
    robot->moveToPoint(-30, 10, 1000, true);
    robot->turnToHeading(345, 500);
    arm->set_target(ALLIANCE_STAKE);
    intake->stop();
    robot->timedMove(20, 2000);
}

void bestautonfr::sawp(Robot* robot) {
    robot->poseSet = true;
    robot->calibrate();
    robot->set_pose_mode(ODOM);

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();
    intake->arm = arm;
    intake->color_sort = false;

    Task updates([&]() {
        while (true) {
            arm->update();
            intake->update();
            delay(1);
        }
    });

    auto stage_1_stop = [&] {
        return intake->detected_ring(STAGE_2);
    };

    // robot->left->set_brake_mode_all(E_MOTOR_BRAKE_BRAKE);
    // robot->right->set_brake_mode_all(E_MOTOR_BRAKE_BRAKE);
    
    robot->set_pose(-23.5, 23.5, 0);

    // robot->turnToHeading(180, 1000);

    // robot->left->set_brake_mode_all(E_MOTOR_BRAKE_COAST);
    // robot->right->set_brake_mode_all(E_MOTOR_BRAKE_COAST);

    // robot->turnToHeading(0, 1000);
    delay(10000000);
    return;
    robot->set_pose(-58.8, 11.3, 270);

    robot->turnToHeading(225, 500);
    arm->set_target(ALLIANCE_STAKE);
    delay(500);

    robot->moveToPoint(-23.5, 23.5, 1500, false, false, 80);
    arm->set_target(LOAD);
    robot->moveToPoint(-20, 23.5, 1000, false, false, 40);
    // robot->timedMove(-20, 500);
    robot->set_mogo(true);

    robot->turnToHeading(0, 1000);
    robot->set_pose(-23.5, 23.5, robot->get_pose().get_degrees());
    delay(100000000);
    return;
    // intake->forwards();

    robot->moveToPoint(-6, 42, 1000, true, false, 60);
    delay(1000);

    return;
    robot->turnToHeading(160, 500);
    robot->moveToPoint(-23.5, 47, 1000, true, false, 80);

}

