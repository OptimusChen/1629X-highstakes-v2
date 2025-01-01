#include "autonomous.hpp"
#include "lib/util.hpp"
#include "lib/controller/pid.hpp"
#include "paths.hpp"
#include "controls.hpp"
#include "intake.hpp"
#include "arm.hpp"

MPConstraint fast{64, 250, INCH};
MPConstraint medium{50, 250, INCH};
MPConstraint huh{60, 200, INCH};
MPConstraint huh2{55, 150, INCH};
MPConstraint slow{40, 250, INCH};
MPConstraint kinda_slow{45, 250, INCH};

#define SPAM 2

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

void bestautonfr::sawp(Robot* robot) {
    robot->set_pose(-55, -15, 90);
    robot->initialize_particle_filter();
    robot->set_pose_mode(MCL);  

    delay(1000);

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();

    arm->set_target(LOAD);

    Task updates([&]() {
        while (true) {
            arm->update();
            intake->update();
            delay(1);
        }
    });

    Distance front(F_DISTANCE);

    robot->turnToHeading(140, 1000);
    robot->moveToPoint(-55, -15, 500, true, false, 60);

    arm->set_target(ALLIANCE_STAKE);

    const int target = 300;
    float error = front.get_distance() - target;
    while (abs(error) > 30) {
        robot->left->move(30 * util::sign(error));
        robot->right->move(30 * util::sign(error));
        error = front.get_distance() - target;

        delay(5);
    }
    robot->left->move(0);
    robot->right->move(0);;

    delay(1000);

    arm->set_target(REST);

    robot->ramsete({{-58.16, -12.38}, {-47.82, -21.27}, {-33.96, -11.55}, {-25.89, -21.89}}, medium, BACKWARDS);
    robot->timedMove(-20, 300);

    robot->set_mogo(true);
    intake->forwards();
    robot->turnToHeading(270, 750);
    robot->moveToPoint(-24.5, -50, 3000, true, false);

    robot->ramsete({{-24.50, -50.00}, {-24.50, -29.75}, {-24.50, -29.75}, {-14.10, -10.31}}, medium, BACKWARDS);
    intake->stop();
    arm->set_target(MID);

    delay(1000);
}

/*

SKILLS

*/

void bestautonfr::skills(Robot* robot) {
    robot->set_pose(-58, 0, 0);
    
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
        auto hue = intake->optical->get_hue();
        return intake->detected_ring(STAGE_1) && (hue > 0 && hue < 30);
    };
    auto stage_2_stop = [&] {
        return intake->detected_ring(STAGE_2);
    };

    intake->forwards();
    delay(500);

    robot->set_pose_mode(MCL);  

    delay(250);

    // CORNER 1

    robot->moveToPoint(-49, robot->get_pose().y, 500, true, false);
    robot->turnToHeading(90, 750);
    robot->moveToPoint(robot->get_pose().x, -23, 500, false, false, 80);
    robot->moveToPoint(robot->get_pose().x, -26, 500, false, false, 30);

    arm->set_target(REST);

    robot->set_mogo(true);

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
    
    robot->ramsete({{-60.00, -60.00}, {3.64, -58.32}, {13.78, -38.51}, {25.91, -50.07}}, {70, 400, INCH});
    delay(1000);

    for (int i = 0; i < SPAM; i++) {
        intake->stop();
        delay(50);
        intake->forwards();
        delay(50);
    }
    intake->stop();

    intake->backwards(40);
    delay(100);
    intake->stop();
    arm->set_target(MID);
    delay(250);

    intake->set_stop_condition(stage_1_stop);
    intake->forwards();

    robot->ramsete({{25.91, -49.88}, {32.92, -57.18}, {37.43, -60.72}, {40.61, -60.72}}, slow);

    robot->ramsete({{40.00, -59.72}, {17.32, -59.90}, {-0.03, -61.40}, {0.00, -52.75}}, fast , BACKWARDS);

    robot->moveToPoint(1, -61, 750, true, false, 60);

    // start logic ws
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
    // end logic ws

    // CORNER 2

    robot->ramsete({{0, -54}, {1.00, -35.67}, {47.14, -32.73}, {59.17, -23.35}}, medium, BACKWARDS);
    robot->timedMove(-30, 500);
    robot->set_mogo(true);  

    intake->forwards(); 
    robot->ramsete({{58.32, -24.02}, {23.69, -24.02}, {3.70, -48.00}, {31.57, -48.00}}, fast);
    arm->set_target(LOAD);
    delay(1000);
    robot->moveToPoint(40, -48, 1000, true, true, 70);
    for (int i = 0; i < SPAM; i++) {
        intake->stop();
        delay(50);
        intake->forwards();
        delay(50);
    }
    intake->stop();
    arm->set_target(MID + 20);
    delay(500);
    // robot->moveToPoint(45, -48, 1000, true, true, 40);
    // delay(1000);
    intake->set_stop_condition(stage_1_stop_color);
    intake->forwards();
    robot->moveToPoint(53, -48, 2000, true, false, 40);

    robot->turnToHeading(135, 750);
    robot->moveToPoint(58, -59, 1000, false, false, 60);
    intake->stop();

    robot->set_mogo(false);
    arm->set_target(LOAD);

    // MIDDLE

    robot->ramsete({{58.00, -59.00}, {47.23, -49.51}, {47.23, -35.86}, {47.23, -18.80}}, medium);
    robot->turnToHeading(270, 750);
    robot->moveToPoint(47, 12, 2000, false, false, 50);
    robot->timedMove(-20, 250);
    robot->set_mogo(true);

    robot->ramsete({{47.00, 12.00}, {47.00, 3.78}, {52.81, 0.00}, {61.81, 0.00}}, medium);
    robot->turnToHeading(0, 500);

    const int target = 300;
    float error = front.get_distance() - target;
    while (abs(error) > 30) {
        robot->left->move(30 * util::sign(error));
        robot->right->move(30 * util::sign(error));
        error = front.get_distance() - target;

        delay(5);
    }
    robot->left->move(0);
    robot->right->move(0);

    arm->set_target(ALLIANCE_STAKE);
    delay(500);
    robot->timedMove(-40, 1000);
    arm->set_target(REST);
    delay(500);
    intake->set_stop_condition(nullptr);
    intake->forwards(100);

    // CORNER 2
    robot->turnToHeading(270, 500);
    intake->set_stop_condition(stage_1_stop);
    robot->ramsete({
        {47.00, 0.00}, {47.00, -17.96}, {43.88, -21.72}, {18.38, -23.13}
    }, huh);

    robot->turnToHeading(125, 250);

    robot->ramsete({
        {17.76, -23.13}, {10.52, -12.50}, {2.60, -12.50}, {2.60, 0.00}
    }, huh);
    intake->forwards(100);
    intake->set_stop_condition(nullptr);
    delay(500);
    intake->set_stop_condition(stage_1_stop);

    robot->turnToHeading(45, 500);
    robot->moveToPoint(23.5, 23.5, 1000, true, false, 100);
    intake->set_stop_condition(nullptr);
    intake->forwards();

    // CORNER 3
    intake->color_sort = true;

    robot->ramsete({{23.50, 23.50}, {32.35, 30.75}, {45.43, 8.77}, {45.43, 37.90}}, huh2);
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
    robot->ramsete({{0.00, 54.00}, {-0.00, 35.52}, {-15.14, 30.38}, {-28.09, 24.09}}, medium);
    robot->turnToHeading(0, 500);
    robot->moveToPoint(-50, 24, 3000, false, false, 50);
    robot->timedMove(-20, 250);
    robot->set_mogo(true);

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