#include "autonomous.hpp"
#include "lib/util.hpp"
#include "lib/controller/pid.hpp"
#include "paths.hpp"
#include "controls.hpp"
#include "intake.hpp"
#include "arm.hpp"

MPConstraint fast{64, 250, INCH};
MPConstraint medium{50, 250, INCH};
MPConstraint huh{60, 150, INCH};
MPConstraint huh2{55, 150, INCH};
MPConstraint slow{40, 250, INCH};
MPConstraint kinda_slow{45, 250, INCH};

#define SPAM 3

float ret(Rotation rot) {
    float measure = rot.get_angle() / 100.0f;
    if (measure > 355) return 0;
    return measure;
}

void bestautonfr::skills(Robot* robot) {
    robot->set_pose(-56, 0, 180);
    robot->initialize_particle_filter();
    robot->set_pose_mode(MCL);  

    delay(1000);

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();
    Distance front(F_DISTANCE);

    intake->arm = arm;
    intake->color_sort = false;
    intake->antijam = false;

    auto stage_1_stop = [&] {
        return intake->detected_ring(STAGE_1);
    };
    auto stage_2_stop = [&] {
        return intake->detected_ring(STAGE_2);
    };

    Task updates([&]() {
        while (true) {
            arm->update();
            intake->update();
            delay(1);
        }
    });

    arm->set_target(ALLIANCE_STAKE);
    delay(1000);

    robot->set_pose_mode(MCL);  

    delay(250);

    // CORNER 1

    robot->moveToPoint(-49, robot->get_pose().y, 1000, false, false, 80);
    robot->turnToHeading(90, 750);
    robot->moveToPoint(robot->get_pose().x, -26, 3000, false, false, 30);

    arm->set_target(REST);

    robot->set_mogo(true);

    delay(500);

    intake->forwards();

    robot->ramsete({
        {-46.34, -25.72}, {-48.09, -9.80}, {14.55, -38.70}, {11.60, -51.21}, 
        {11.60, -51.21}, {8.06, -66.20}, {-4.84, -40.11}, {-50.33, -47.50}, 
        {-50.33, -47.50}, {-71.14, -50.88}, {-57.99, -64.19}, {-30.00, -58.60}
    }, huh);

    robot->moveToPoint(-60, -60, 1000, false, false, 60);

    robot->set_rush_arm(true);
    delay(500);
    robot->set_rush_arm(false);

    robot->set_mogo(false);

    // WALL STAKE

    intake->forwards();
    arm->set_target(LOAD);
    
    robot->ramsete({{-60.00, -60.00}, {3.64, -58.32}, {13.78, -38.51}, {25.91, -50.07}}, fast);
    delay(500);

    for (int i = 0; i < SPAM; i++) {
        intake->stop();
        delay(100);
        intake->forwards();
        delay(100);
    }
    intake->stop();

    intake->backwards(40);
    delay(100);
    intake->stop();
    arm->set_target(MID);
    delay(500);

    intake->set_stop_condition(stage_1_stop);
    intake->forwards();

    robot->turnToHeading(317, 750);
    robot->ramsete({{25.91, -49.88}, {32.92, -57.18}, {37.43, -60.72}, {40.61, -60.72}}, slow);

    robot->ramsete({{40.00, -59.72}, {17.32, -59.90}, {-0.03, -61.40}, {0.00, -52.75}}, medium , BACKWARDS);

    robot->moveToPoint(1, -61, 750, true, false, 60);

    robot->timedMove(40, 500);
    arm->set_target(SCORE);
    delay(500);
    robot->timedMove(-40, 1000);

    arm->set_target(LOAD);
    robot->timedMove(40, 500);
    intake->set_stop_condition(nullptr);
    intake->forwards();
    robot->timedMove(40, 1000);
    delay(1000);
    for (int i = 0; i < SPAM; i++) {
        intake->stop();
        delay(100);
        intake->forwards();
        delay(100);
    }
    intake->stop();
    arm->set_target(SCORE);
    robot->timedMove(40, 500);
    robot->timedMove(-40, 1000);
    robot->moveToPoint(0, -54, 500, true, false, 40);

    // CORNER 2

    robot->ramsete({{1.00, -54.00}, {1.00, -35.67}, {47.14, -32.73}, {59.17, -23.35}}, medium, BACKWARDS);
    robot->timedMove(-30, 500);
    robot->set_mogo(true);  

    intake->forwards(); 
    robot->ramsete({{58.32, -24.02}, {23.69, -24.02}, {3.70, -48.00}, {31.57, -48.00}}, medium);
    arm->set_target(LOAD);
    delay(250);
    robot->moveToPoint(40, -48, 1000, true, true, 40);
    delay(2000);
    for (int i = 0; i < SPAM; i++) {
        intake->stop();
        delay(100);
        intake->forwards();
        delay(100);
    }
    intake->stop();
    arm->set_target(MID + 20);
    delay(500);
    intake->forwards();
    robot->moveToPoint(45, -48, 1000, true, true, 40);
    delay(1000);
    intake->set_stop_condition(stage_1_stop);
    intake->forwards();
    robot->moveToPoint(54, -48, 1000, true, false, 40);

    robot->turnToHeading(135, 750);
    robot->moveToPoint(58, -59, 2000, false, false, 60);

    robot->set_mogo(false);
    arm->set_target(LOAD);

    // MIDDLE

    robot->ramsete({{58.00, -59.00}, {47.23, -49.51}, {47.23, -35.86}, {47.23, -18.80}}, medium);
    robot->turnToHeading(270, 750);
    robot->moveToPoint(47, 12, 2000, false, false, 40);
    robot->set_mogo(true);
    intake->forwards();

    robot->ramsete({{47.00, 12.00}, {47.00, 3.78}, {52.81, 0.00}, {61.81, 0.00}}, medium);
    robot->turnToHeading(0, 500);

    const int target = 300;
    float error = front.get_distance() - target;
    while (abs(error) > 30) {
        robot->left->move(20 * util::sign(error));
        robot->right->move(20 * util::sign(error));
        error = front.get_distance() - target;

        delay(5);
    }
    robot->left->move(0);
    robot->right->move(0);

    arm->set_target(ALLIANCE_STAKE);
    delay(500);
    robot->timedMove(-40, 1000);
    arm->set_target(REST);
    intake->set_stop_condition(nullptr);
    intake->forwards(100);

    // CORNER 2
    robot->turnToHeading(270, 750);
    intake->set_stop_condition(stage_1_stop);
    robot->ramsete({
        {47.00, 0.00}, {48.46, -33.68}, {29.32, -28.52}, {18.52, -20.41}
    }, huh2);

    robot->ramsete({
        {18.52, -20.41}, {9.42, -13.59}, {2.60, -12.46}, {2.60, -0.14}
    }, huh2);
    intake->forwards(80);
    intake->set_stop_condition(nullptr);
    delay(500);
    intake->set_stop_condition(stage_1_stop);

    robot->turnToHeading(45, 500);
    robot->moveToPoint(23.5, 23.5, 1000, true, false, 100);
    intake->set_stop_condition(nullptr);
    intake->forwards();

    // CORNER 3
    intake->color_sort = true;

    robot->ramsete({{23.50, 23.50}, {32.35, 30.75}, {45.43, 8.77}, {45.43, 38.90}}, huh2);
    robot->moveToPoint(45, 48, 1000, true, false, 60);
    robot->moveToPoint(45, 60, 2000, true, false, 40);
    robot->moveToPoint(45, 36, 1000, false, false, 60);

    arm->set_target(LOAD);
    delay(500);
    robot->moveToPoint(55, 44, 1000, true, false, 60);

    robot->turnToHeading(225, 500);
    robot->moveToPoint(60, 60, 1000, false, false, 80);
    delay(1000);

    robot->set_mogo(false);
    intake->stop();

    arm->set_target(MID);
    delay(1000);
    intake->backwards();

    robot->ramsete({{60.00, 60.00}, {60.00, 40.18}, {7.04, 47.76}, {-9.71, 47.76}}, medium);

    intake->stop();

    // WALL STAKE 2

    robot->moveToPoint(0, 47, 1000, false, false, 80);
    robot->turnToHeading(90, 500);
    robot->moveToPoint(0, 61, 2000, true, false, 80);

    robot->timedMove(40, 500);
    arm->set_target(SCORE);
    delay(500);
    robot->timedMove(-40, 1000);

    arm->set_target(LOAD);
    robot->timedMove(40, 1500);
    delay(500);
    intake->set_stop_condition(nullptr);
    intake->forwards();
    delay(1000);
    for (int i = 0; i < SPAM; i++) {
        intake->stop();
        delay(100);
        intake->forwards();
        delay(100);
    }
    intake->stop();
    arm->set_target(SCORE);
    robot->timedMove(40, 500);
    robot->timedMove(-20, 500);
    robot->moveToPoint(0, 54, 2000, false, false, 40);
    arm->set_target(REST);
    delay(500);

    // CORNER 4

    robot->turnToHeading(270, 500);
    intake->set_stop_condition(stage_1_stop);
    intake->forwards(80);
    robot->ramsete({{0.00, 54.00}, {-0.00, 35.52}, {-15.14, 30.38}, {-28.09, 24.09}}, medium);
    robot->turnToHeading(0, 500);
    robot->moveToPoint(-50, 24, 2000, false, false, 60);
    robot->set_mogo(true);

    intake->set_stop_condition(nullptr);
    intake->forwards();

    robot->ramsete({
        {-50.56, 23.33}, {-28.09, 23.33}, {-12.47, 20.66}, {-2.57, 41.80},
        {-2.57, 41.80}, {4.68, 57.28}, {-22.74, 46.56}, {-48.08, 46.56},
        {-48.08, 46.56}, {-73.43, 46.56}, {-43.89, 60.27}, {-33.80, 60.27}
    }
    , huh);

    robot->moveToPoint(-60, 60, 1000, false, false, 60);

    return;

    robot->ramsete({{robot->get_pose().x, robot->get_pose().y}, {0.14, -19.07}, {32.03, -45.06}, {57.94, -24.39}}, medium, BACKWARDS);
    intake->forwards();

    delay(1000);
    intake->stop();

    
    arm->set_target(LOAD);


    intake->backwards(20);
    delay(200);
    intake->stop();
    arm->set_target(MID + 10);
    intake->set_stop_condition(stage_1_stop);
    intake->forwards(80);
    // robot->turnToHeading(225, 500);
    robot->set_mogo(false);

    // robot->ramsete({{58.00, -58.00}, {33.68, -44.29}, {47.52, -35.39}, {47.52, -17.95}}, medium);

    delay(100000);

    return;
    

    // MIDDLE

    // robot->moveToPoint(46.5, -24, 2000, true, true, 100);


    // robot->ramsete({{56.99, -23.79}, {50.17, -29.57}, {54.30, -37.80}, {47.65, -43.62}}, huh);
    // delay(1500);
    // for (int i = 0; i < SPAM; i++) {
    //     intake->stop();
    //     delay(100);
    //     intake->forwards();
    //     delay(100);
    // }
    // intake->stop();
    // arm->set_target(MID);
    // robot->ramsete({{47.65, -43.62}, {54.30, -37.80}, {50.17, -29.57}, {56.99, -23.79}}, huh, BACKWARDS);

    // intake->set_stop_condition(stage_1_stop);
    // intake->forwards(80);

    // robot->turnToHeading(290, 750);
    // robot->ramsete({
    //     {57.28, -24.09}, {62.63, -34.56}, {57.28, -46.32}, {53.35, -47.00}, 
    //     {53.35, -47.00}, {41.20, -49.10}, {45.92, -41.10}, {42.67, -37.80}
    // }, huh2);

    // delay(1000);


    robot->turnToHeading(270, 750);

    robot->set_mogo(true);

    robot->turnToHeading(0, 500);

    // float error = front.get_distance() - 400;
    // while (error > 30) {
    //     robot->left->move(20 * util::sign(error));
    //     robot->right->move(20 * util::sign(error));
    //     error = front.get_distance() - 400;

    //     delay(5);
    // }
    // robot->left->move   (0);
    // robot->right->move(0);
    
    arm->set_target(ALLIANCE_STAKE);
    delay(10000);
    
    return;
    intake->set_stop_condition(nullptr);
    intake->backwards(-20);
    delay(500);
    intake->stop();
    delay(500);
    intake->forwards(80);
    robot->timedMove(-20, 500);

    arm->set_target(REST);
    robot->moveToPoint(47, 0, 1000, false, false, 60);
    robot->turnToHeading(270, 750);
    intake->set_stop_condition(stage_1_stop);
    robot->ramsete({
        {47.00, 0.00}, {48.46, -33.68}, {29.32, -28.52}, {18.52, -20.41}
    }, huh);

    robot->ramsete({
        {18.52, -20.41}, {9.42, -13.59}, {2.60, -12.46}, {2.60, -0.14}
    }, huh);
    intake->forwards(80);
    intake->set_stop_condition(nullptr);
    delay(500);
    intake->set_stop_condition(stage_1_stop);

    robot->turnToHeading(45, 500);
    robot->moveToPoint(23.5, 23.5, 2000, true, false, 100);
    intake->set_stop_condition(nullptr);
    intake->forwards();

    delay(1000000);

    return;
}