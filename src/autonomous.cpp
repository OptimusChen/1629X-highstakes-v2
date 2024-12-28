#include "autonomous.hpp"
#include "lib/util.hpp"
#include "lib/controller/pid.hpp"
#include "paths.hpp"
#include "controls.hpp"
#include "intake.hpp"
#include "arm.hpp"

MPConstraint fast{64, 300, INCH};
MPConstraint medium{50, 200, INCH};
MPConstraint huh{64, 200, INCH};

MPConstraint slow{30, 300, INCH};

float ret(Rotation rot) {
    float measure = rot.get_angle() / 100.0f;
    if (measure > 355) return 0;
    return measure;
}

void bestautonfr::skills(Robot* robot) {
    // delay(3000);
    // robot->set_pose_mode(MCL);  

    // robot->ramsete({{-56.00, 0.00}, {-27.17, 1.57}, {-47.83, -35.77}, {-47.83, -47.33}}, fast);

    // return;

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();

    Task updates([&]() {
        while (true) {
            arm->update();
            intake->update();
            delay(10);
        }
    });

    intake->forwards();

    delay(1000);

    intake->stop();

    robot->set_pose_mode(MCL);  

    pros::delay(250);

    robot->moveToPoint(-47, robot->get_pose().y, 750, true, false);

    robot->turnToHeading(90, 500);
    robot->moveToPoint(robot->get_pose().x, -27, 2500, false, false, 30);

    robot->set_mogo(true);
    
    intake->forwards();

    robot->ramsete({
        {-46.34, -25.72}, {-48.095, -9.802}, {14.55, -38.70}, {11.60, -51.21}, 
        {11.60, -51.21}, {8.06, -66.20}, {-3.46, -38.70}, {-48.95, -46.09}, 
        {-48.95, -46.09}, {-67.40, -49.09}, {-57.99, -64.19}, {-30.00, -58.60}
    }, huh);

    robot->moveToPoint(-60, -60, 1000, false, false, 60);

    robot->set_mogo(false);

    intake->stop();
    intake->forwards(80);
    intake->set_stop_condition([&] {
        return intake->detected_ring();
    });

    robot->ramsete({{-60.00, -60.00}, {3.64, -58.32}, {20.13, -41.64}, {46.85, -48.27}}, fast);
    robot->ramsete({{46.85, -48.27}, {31.69, -42.11}, {-0.15, -68.46}, {0, -47.61}}, medium, Direction::BACKWARDS);

    robot->moveToPoint(0, -56, 1000, true, false, 40);

    robot->left->move(40);
    robot->right->move(40);
    delay(1000);
    robot->left->move(0);
    robot->right->move(0);

    intake->stop();    
    intake->set_stop_condition(nullptr);

    arm->set_target(SCORE);

    delay(500);

    intake->forwards();

    delay(1500);

    for (int i = 0; i < 4; i++) {
        intake->stop();
        delay(100);
        intake->forwards();
        delay(100);
    }

    intake->stop();

    arm->set_target(DESCORE);

    delay(500);

    robot->left->move(-40);
    robot->right->move(-40);
    delay(500);
    robot->left->move(0);
    robot->right->move(0);

    arm->set_target(SCORE);
    
    delay(500);

    intake->forwards();

    delay(1000);

    robot->left->move(40);
    robot->right->move(40);
    delay(1000);
    robot->left->move(0);
    robot->right->move(0);

    for (int i = 0; i < 4; i++) {
        intake->stop();
        delay(100);
        intake->forwards();
        delay(100);
    }

    intake->stop();

    arm->set_target(DESCORE);

    delay(1000);

    arm->set_target(SCORE);

    delay(5000);
}