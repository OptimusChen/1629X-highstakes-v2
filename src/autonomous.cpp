#include "autonomous.hpp"
#include "lib/util.hpp"
#include "lib/controller/pid.hpp"
#include "paths.hpp"
#include "controls.hpp"
#include "intake.hpp"
#include "arm.hpp"

MPConstraint fast{64, 250, INCH};
MPConstraint medium{50, 250, INCH};
MPConstraint huh{55, 250, INCH};
MPConstraint huh2{40, 250, INCH};
MPConstraint slow{40, 250, INCH};
MPConstraint kinda_slow{45, 250, INCH};

#define SPAM 2

float ret(Rotation rot) {
    float measure = rot.get_angle() / 100.0f;
    if (measure > 355) return 0;
    return measure;
}

void bestautonfr::skills(Robot* robot) {
    // robot->set_pose(0, 0, 90);
    // robot->ramsete({
    //     {0.00, 0.00}, {0.00, 30.00}, {0.00, 30.00}, {30.00, 30.00}, 
    //     {30.00, 30.00}, {54.72, 30.00}, {12.85, 0.19}, {-0.00, 0.00}
    // }, fast);
    // return;

    robot->set_pose(-50, 0, 180);
    robot->initialize_particle_filter();
    robot->set_pose_mode(MCL);  

    delay(2000);

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();
    Distance front(F_DISTANCE);

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
            delay(10);
        }
    });

    // arm->set_target(ALLIANCE_STAKE);
    delay(1000);

    robot->set_pose_mode(MCL);  

    delay(250);

    // CORNER 1

    // robot->ramsete({{-50.84, -0.14}, {-35.98, 0.50}, {-46.98, -12.01}, {-46.80, -22.75}}, slow, BACKWARDS);
    robot->moveToPoint(-47.5, robot->get_pose().y, 500, false, false, 60);
    robot->turnToHeading(90, 750);
    robot->moveToPoint(robot->get_pose().x, -25, 1500, false, false, 20);

    arm->set_target(REST);

    robot->set_mogo(true);

    delay(5000);

    return;

    intake->forwards();

    robot->ramsete({
        {-46.34, -25.72}, {-48.09, -9.80}, {14.55, -38.70}, {11.60, -51.21}, 
        {11.60, -51.21}, {8.06, -66.20}, {-3.46, -38.70}, {-48.95, -46.09}, 
        {-48.95, -46.09}, {-69.76, -49.48}, {-57.99, -64.19}, {-30.00, -58.60}
    }, huh);

    robot->moveToPoint(-60, -60, 1000, false, false, 100);
    return;

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
    intake->forwards(100);

    robot->turnToHeading(317, 750);
    robot->ramsete({{25.91, -49.88}, {32.92, -57.18}, {37.43, -60.72}, {55.61, -60.72}}, slow);

    robot->ramsete({{55.00, -59.72}, {17.32, -59.90}, {-0.03, -61.40}, {0.00, -52.75}}, medium , BACKWARDS);

    robot->moveToPoint(2, -61, 2000, true, false, 60);

    robot->timedMove(40, 500);
    arm->set_target(SCORE);
    delay(500);
    robot->timedMove(-40, 1000);

    arm->set_target(LOAD);
    robot->timedMove(40, 1500);
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
    robot->timedMove(-40, 500);

    arm->set_target(REST);

    // CORNER 2

    arm->set_target(REST);
    robot->ramsete({{robot->get_pose().x, robot->get_pose().y}, {0.14, -19.07}, {32.03, -45.06}, {57.94, -24.39}}, medium, BACKWARDS);
    intake->forwards();

    robot->set_mogo(true);    
    delay(1000);
    intake->stop();

    intake->forwards();
    arm->set_target(LOAD);

    robot->ramsete({{58.32, -24.02}, {23.69, -24.02}, {3.70, -48.00}, {31.57, -48.00}}, medium);
    robot->timedMove(127, 700);

    robot->turnToHeading(90, 750);
    intake->backwards(20);
    delay(200);
    intake->stop();
    robot->moveToPoint(60, -60, 2000, false, false, 60);
    arm->set_target(MID);
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

    float error = front.get_distance() - 400;
    while (error > 30) {
        robot->left->move(20 * util::sign(error));
        robot->right->move(20 * util::sign(error));
        error = front.get_distance() - 400;

        delay(5);
    }
    robot->left->move(0);
    robot->right->move(0);
    
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