#include "autonomous.hpp"
#include "lib/util.hpp"
#include "lib/controller/pid.hpp"
#include "paths.hpp"
#include "controls.hpp"
#include "intake.hpp"
#include "arm.hpp"
#include "lib/logging.hpp"

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
    bool runningAuton = true;

    robot->set_pose_mode(MCL);

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();
    Distance front(F_DISTANCE);
    Distance left(L_DISTANCE);
    Distance right(R_DISTANCE);
    Distance back(B_DISTANCE);

    intake->arm = arm;
    intake->color_sort = true;
    intake->antijam = true;
    intake->set_color(RED);

    Task updates([&]() {
        while (runningAuton) {
            arm->update();
            delay(1);
        }
    });

    Task intakeupdates([&]() {
        while (runningAuton) {
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

    robot->set_use_slow_angular(false);

    intake->forwards();

    pros::delay(250);
    intake->stop();

    const int fast_speed = 100;
    const int mid_speed = 90;

    //-----------CLAMP FIRST MOGO------------------//

    robot->moveToPoint(-49, 0, 900, true, false, fast_speed);

    robot->turnToHeading(270, 500);

    robot->moveToPoint(-46.5, 10, 500, false, false, 127, true);
    robot->moveToPoint(-46.5, 21, 500, false, false, 60);

    robot->set_mogo(true);
    delay(200);
    //-----------SCORE ONE RING------------------//

    robot->turnToHeading(0, 500);

    intake->forwards();

    robot->moveToPoint(-26, 23, 700, true, false, 127);

    //-----------OBTAIN 2nd RING------------------//

    Task lbdelay1([&] {
        delay(1200);
        arm->set_target(LOAD);
    });

    robot->turnToHeading(33, 500);

    robot->moveToPoint(28, 44.5, 1400, true, false, 127);

    // robot->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    robot->moveToPoint(0, 38, 1000, false, true, mid_speed);

    //-----------LADY BROWN 1------------------//

    robot->turnToHeading(90, 500);
    intake->stop();
    arm->set_target(MID + 40);
    delay(200);
    intake->forwards();
    robot->moveToPoint(0, 62, 1000, true, true, fast_speed);
    // robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    arm->liftPID.kP = 100;
    arm->set_target(SCORE + 50);
    arm->liftPID.kP = 1.5;
    robot->timedMove(40, 400);

    //-----------SCORE THREE RINGS------------------//

    robot->moveToPoint(0, 47, 750, false, false, fast_speed, true);
    arm->set_target(REST);

    robot->moveToPoint(-40, 47, 1000, true, false, fast_speed);
    robot->moveToPoint(-58, 47, 1000, true, false, 60);
    delay(250);
    robot->moveToPoint(-43, 60, 1500, true, true, mid_speed);
    // robot->timedMove(40, 500);

    //-----------Place in corner------------------//

    intake->antijam = false;
    // robot->moveToPoint(-64, 58, 1000, false, false, 90, true);
    robot->turnToHeading(345, 500);
    robot->set_mogo(false);
    robot->moveToPoint(-64, robot->get_pose().y, 500, false, false, 80, true);

    Task intakedelay0([&] {
        delay(400);
        intake->forwards(80);
    });

    ///////////////////////////////     ////////////////
    //--------------2ND SECTION----------------//
    ///////////////////////////////////////////////


    //Get ring for alliance stake

    intake->antijam = true;
    robot->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    // robot->moveToPoint(38, 46, 1700, true, true, 100);
    intake->set_stop_condition(stage_1_stop_color);
    robot->moveToPoint(42, 46, 2000, true, true, 127);
    robot->set_rush_arm_right(true);

    delay(200);

    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);
    
    //Clamp second goal

    arm->set_target(LOAD);
    robot->turnToHeading(130, 1000, false, 0, 127);
    robot->moveToPoint(58, 23, 1200, false, true, fast_speed);
    // robot->timedMove(-20, 500);
    robot->set_rush_arm_right(false);
    robot->set_mogo(true);

    intake->set_stop_condition(nullptr);
    intake->forwards();
    delay(200);

    //put in corner
    robot->set_rush_arm_right(true);
    robot->turnToHeading(70, 500);
    intake->stop();

    robot->moveToPoint(62, 51, 1000, true, true, 127);

    Task doinkerdelay1([&] {
        delay(200);
        robot->set_rush_arm_right(false);
    });
    robot->turnToHeading(210, 1000);

    robot->set_mogo(false);
    robot->moveToPoint(68, 64, 1000, false, true, 127);
 
    intake->stop();

    //get third goal
    robot->moveToPoint(48, 30, 500, true, true, fast_speed);
    
    arm->set_target(MID + 5);
    intake->stop();
    robot->turnToHeading(90, 500);
    
    intake->backwards();
    robot->moveToPoint(47, 10, 1000, false, false, mid_speed);
    robot->moveToPoint(47, 0, 500, false, false, mid_speed - 30);
    intake->stop();
    robot->set_mogo(true);

    robot->turnToHeading(90, 300);
    // robot->moveToPoint(47, 0, 500, true, false, mid_speed);

    //score alliance stake

    int target = 1725;
    float error = front.get_distance() - target;
    while (abs(error) > 5) {
        float power = error * 0.2;
        if (abs(power) < 15) {
            power = util::sign(power) * 15;
        }
        robot->left->move(power);
        robot->right->move(power);
        error = front.get_distance() - target;

        delay(5);
    }
    robot->left->move(0);
    robot->right->move(0);

    robot->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    robot->turnToHeading(0, 750);
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    arm->liftPID.kP = 1;
    arm->set_target(MID + 20);
    arm->liftPID.kP = 1.5;

    target = 380;
    error = front.get_distance() - target;
    while (abs(error) > 20) {
        float power = util::sign(error) * 20;
        robot->left->move(power);
        robot->right->move(power);
        error = front.get_distance() - target;

        delay(5);
    }
    robot->left->move(0);
    robot->right->move(0);

    arm->liftPID.kP = 100;
    arm->set_target(ALLIANCE_STAKE + 50);
    arm->liftPID.kP = 1.5;
    delay(400);

    intake->forwards();
    intake->color_sort = true;
    robot->moveToPoint(40, robot->get_pose().y, 1000, false, false, 80);
    arm->set_target(REST);

    ///////////////////////////////////////////////
    //--------------3RD SECTION----------------//
    ///////////////////////////////////////////////

    //score one ring
    robot->moveToPoint(48, robot->get_pose().y, 500, true, false, fast_speed);
    robot->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    robot->turnToPoint(22, 22, 500);
    robot->moveToPoint(22, 22, 1500, true, true, fast_speed);

    //cross under ladder score 3
    // robot->turnToPoint(-47, -47, 500);
    robot->turnToHeading(225, 500);
    intake->stop();
    
    bool under = true;
    Task intakeunderbar([&] {
        while (under) {
            Pose pose = robot->get_pose();
            int distanceFrom00 = sqrt(pow(pose.x, 2) + pow(pose.y, 2));
            if (distanceFrom00 < 5) {
                intake->forwards();
            } else {
                intake->stop();
            }
            delay(10);
        }
    });
    robot->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    robot->moveToPoint(-20, -20, 1000, true, true, fast_speed);
    under = false;
    Task intakedelay1([&] {
        delay(500);
        intake->forwards();
    });
    // SCORE 3 RINGS
    robot->moveToPoint(-41, -41, 1500, true, true, mid_speed - 15);
    robot->turnToHeading(180, 500);
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);
    robot->moveToPoint(-56, -47, 1000, true, false, mid_speed);
    delay(250);
    robot->moveToPoint(-40, -60, 1500, true, true, mid_speed);

    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);
    // delay(500);

    //-----------Place in corner------------------//
    robot->moveToPoint(-60, -63, 1250, false, true, fast_speed);
    robot->particleFilter->setNoiseNextUpdate(true);
    robot->set_mogo(false);
    delay(500);

    ///////////////////////////////////////////////
    //--------------4th SECTION------------------//
    ///////////////////////////////////////////////
    robot->moveToPoint(-46, -45, 500, true, true, fast_speed);
    intake->stop();
    robot->moveToPoint(-47, -21, 2000, false, true, mid_speed);
    robot->set_mogo(true);
    delay(300);

    arm->set_target(LOAD);
    intake->forwards();
    robot->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    robot->moveToPoint(-23, -50, 1200, true, true, fast_speed);
    // robot->particleFilter->setNoiseNextUpdate(true);

    //lady brown stuff
    robot->moveToPoint(-1, -40, 1300, false, true, fast_speed);
    intake->stop();
    arm->set_target(MID + 20);

    //-----------LADY BROWN 1------------------//
    Task intakedelay2([&] {
        delay(500);
        intake->forwards();
    });
    robot->turnToHeading(270, 450);
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    robot->moveToPoint(0, -61, 1000, true, true, mid_speed);

    arm->liftPID.kP = 100;
    arm->set_target(SCORE + 50);
    arm->liftPID.kP = 1.5;
    robot->timedMove(40, 500);

    robot->moveToPoint(robot->get_pose().x, -44, 1000, false, false, fast_speed);
    arm->set_target(REST);

    // now go score rings
    robot->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    robot->moveToPoint(24, -48, 850+500, true, true, fast_speed);
    delay(250);
    robot->moveToPoint(25.5, -16, 1500, true, true, fast_speed);
    delay(250);

    robot->moveToPoint(24, -58, 1200, false, true);
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    // score 3 rings
    robot->moveToPoint(44, -47, 1000, true, false, mid_speed);
    delay(500);
    robot->moveToPoint(56, -47, 750, true, false, fast_speed);
    robot->moveToPoint(44, -40, 500, false, false, mid_speed);

    //score two and corner
    robot->turnToHeading(270, 500);
    robot->moveToPoint(44, -56, 750, true, true, mid_speed);

    pros::delay(250);

    //put in corner

    // robot->moveToPoint(38, -43, 1000, true, false, 60);
    robot->turnToHeading(170, 500, false, 0, 60);

    robot->set_mogo(false);
    delay(300);
    robot->moveToPoint(64, -64, 1000, false, false, 127);
    intake->stop();

    // HANG!!!
    intake->hooks.brake();
    arm->set_target(SCORE);
    robot->set_color_sort_piston(false);
    robot->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    robot->moveToPoint(27, -27, 1000, true);
    robot->turnToHeading(315, 750, false, 0, mid_speed);
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);
    robot->timedMove(-100, 2000);

    intake->stop();
    runningAuton = false;
    dump_all();    

}

void bestautonfr::red_rush(Robot* robot, lemlib::Chassis* chassis) {
    bool runningAuton = true;

    robot->set_pose_mode(MCL);

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();
    intake->arm = arm;

    Distance front(F_DISTANCE);

    intake->set_color(RED);

    intake->color_sort = true;

    Task updates([&]() {
        while (runningAuton) {
            arm->update();
            intake->update();
            delay(1);
        }
    });

    auto stage_1_stop = [&] {
        return intake->detected_ring(STAGE_2);
    };

    delay(1000000);

    robot->set_rush_arm_left(true);
    intake->antijam = false;
    intake->forwards(100);
    intake->set_stop_condition(stage_1_stop);
    robot->lateral->kP = 8;
    robot->moveToPoint(-11, 42, 1300, true, false, 127);

    robot->moveToPoint(-23.5, 23.5, 1250, false, true, 80);
    robot->set_mogo(true);

    robot->set_rush_arm_left(false);
    delay(500);

    intake->antijam = true;

    intake->set_stop_condition(nullptr);
    intake->forwards();

    robot->moveToPoint(-23.5, 50, 1000, true, false, 100);
    delay(500);

    robot->turnToHeading(180, 500);
    robot->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    robot->moveToPoint(-55, 55, 800, true, false);
    robot->turnToHeading(135, 500);
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    robot->timedMove(50, 800);
    delay(400);
    robot->timedMove(-40, 400);
    robot->set_lift_intake(true);
    robot->timedMove(50, 800);
    delay(400);
    robot->set_lift_intake(false);

    robot->lateral->kP = 6;

    robot->moveToPoint(-52, 52, 700, false, false);
    robot->turnToHeading(270, 500);
    arm->set_target(LOAD);
    robot->lateral->kP = 4;
    robot->particleFilter->setAddNoise(false);
    robot->moveToPoint(-45, 23, 1250, true, true);
    arm->set_target(MID + 20);
    robot->set_lift_intake(true);
    robot->moveToPoint(-45, 0, 1000, true, true, 127);
    robot->set_lift_intake(false);
    delay(500);
    robot->turnToHeading(190, 1000, false, 0, 60);
    robot->moveToPoint(robot->get_pose().x + 5, robot->get_pose().y, 500, true, false, 80, true);
    arm->set_target(ALLIANCE_STAKE + 50);
    delay(1000);

    runningAuton = false;
}

void bestautonfr::blue_rush(Robot* robot, lemlib::Chassis* chassis) {
    bool runningAuton = true;

    robot->set_pose_mode(MCL);

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();
    intake->arm = arm;

    Distance front(F_DISTANCE);

    intake->set_color(BLUE);

    intake->color_sort = true;

    Task updates([&]() {
        while (runningAuton) {
            arm->update();
            intake->update();
            delay(1);
        }
    });

    auto stage_1_stop = [&] {
        return intake->detected_ring(STAGE_2);
    };

    robot->set_pose(51, 20, 152);

    robot->set_rush_arm_right(true);
    intake->antijam = false;
    intake->forwards(100);
    intake->set_stop_condition(stage_1_stop);
    robot->lateral->kP = 8;
    robot->moveToPoint(11, 42, 1300, true, false, 127, true);
    robot->particleFilter->setNoiseNextUpdate(true);

    robot->moveToPoint(23.5, 20, 1250, false, true, 80);
    robot->set_mogo(true);
    robot->turnToHeading(90, 500);

    robot->set_rush_arm_right(false);
    delay(500);

    intake->antijam = true;

    intake->set_stop_condition(nullptr);
    intake->forwards();

    robot->moveToPoint(23.5, 52, 1000, true, false, 100);
    delay(500);

    robot->turnToHeading(0, 500);
    robot->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    robot->moveToPoint(49, 52, 800, true, false);
    robot->turnToHeading(45, 500);
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    robot->timedMove(50, 800);
    delay(400);
    robot->timedMove(-40, 400);
    robot->set_lift_intake(true);
    robot->timedMove(50, 800);
    delay(400);
    robot->set_lift_intake(false);

    robot->lateral->kP = 6;

    robot->moveToPoint(52, 52, 700, false, false);
    robot->turnToHeading(270, 500);
    arm->set_target(LOAD);
    robot->lateral->kP = 4;
    robot->particleFilter->setAddNoise(false);
    robot->moveToPoint(45, 23, 1250, true, true);
    arm->set_target(MID + 20);
    robot->set_lift_intake(true);
    robot->moveToPoint(45, 0, 1000, true, true, 127);
    robot->set_lift_intake(false);
    delay(500);
    robot->turnToHeading(-10, 1000, false, 0, 60);
    arm->set_target(ALLIANCE_STAKE + 50);

    runningAuton = false;
}

void bestautonfr::red_sawp(Robot* robot) {
    bool runningAuton = true;

    robot->set_pose_mode(MCL);

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();
    intake->arm = arm;
    intake->set_color(RED);
    intake->color_sort = true;

    Task updates([&]() {
        while (runningAuton) {
            arm->update();
            intake->update();
            delay(1);
        }
    });

    auto stage_1_stop = [&] {
        return intake->detected_ring(STAGE_2);
    };

    arm->set_target(ALLIANCE_STAKE + 50);
    delay(500);

    robot->moveToPoint(-48, 23, 500, false, false);
    arm->set_target(REST);
    robot->moveToPoint(-20, 23, 1250, false, true, 60);
    
    robot->set_mogo(true);
    delay(200);

    robot->turnToHeading(50, 500);
    intake->forwards();
    robot->moveToPoint(-5, 40, 850, true, false);
    robot->moveToPoint(-5, 52, 700, true, false);

    robot->moveToPoint(-24, 24, 700, false, false);
    intake->set_stop_condition(stage_1_stop);
    robot->moveToPoint(-24, 50, 1000, true);

    robot->turnToHeading(225, 500);
    intake->set_stop_condition(nullptr);
    intake->forwards();
    robot->set_lift_intake(true);
    robot->moveToPoint(-40, 12, 750, true, false);
    intake->set_stop_condition(stage_1_stop);
    robot->set_mogo(false);
    robot->moveToPoint(-50, 4, 1000, true, false, 60);

    robot->set_lift_intake(false);
    arm->set_target(MID + 80);
    
    robot->turnToHeading(135, 1000, false, 0, 40);
    robot->moveToPoint(-20, -23, 1000, false, true, 80);
    robot->set_mogo(true);
    delay(200);
    robot->turnToHeading(270, 500);
    intake->set_stop_condition(nullptr);
    intake->forwards();
    robot->moveToPoint(-23, -43, 1000, true, true);
    robot->particleFilter->setAddNoise(false);
    robot->moveToPoint(-24, -20, 1000, false, false);
    robot->turnToHeading(45, 500);
    robot->timedMove(40, 500);
    intake->stop();

    delay(500);
    runningAuton = false;
    return;
}

void bestautonfr::blue_sawp(Robot* robot) {
    bool runningAuton = true;

    robot->set_pose_mode(ODOM);

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();
    intake->arm = arm;
    intake->color_sort = true;

    Task updates([&]() {
        while (runningAuton) {
            arm->update();
            intake->update();
            delay(1);
        }
    });

    auto stage_1_stop = [&] {
        return intake->detected_ring(STAGE_2);
    };

    robot->set_pose(58.8, 11.3, 270);

    robot->turnToHeading(315, 500);
    arm->set_target(ALLIANCE_STAKE + 25);
    delay(500);

    robot->moveToPoint(23.5, 23.5, 1000, false, false, 80);
    arm->set_target(REST);
    robot->moveToPoint(19, 23.5, 500, false, false, 40);
    // robot->timedMove(-20, 500);
    robot->set_mogo(true);
    delay(200);

    robot->turnToHeading(180, 500);
    robot->set_pose(23.5, 23.5, robot->get_pose().get_degrees());
    intake->forwards();

    robot->moveToPoint(3, 40, 1500, true, true, 60);
    // delay(300);

    robot->moveToPoint(19, 28, 750, false, false, 80);

    // robot->turnToHeading(160, 500);
    robot->moveToPoint(21.5, 48, 1000, true, true, 80);
    delay(300);

    // get stack
    robot->moveToPoint(39, 18, 2000, true, true, 100);
    robot->set_mogo(false);
    robot->set_lift_intake(true);
    delay(200);
    intake->set_stop_condition(stage_1_stop);
    robot->timedMove(40, 500);
    robot->set_lift_intake(false);
    robot->timedMove(-40, 500);

    robot->set_pose(43.5, 6, robot->get_pose().get_degrees());

    robot->moveToPoint(48, -23, 1000, true, true, 100);
    robot->moveToPoint(23.5, -26, 1000, false, true, 80);
    robot->timedMove(-40, 300);
    robot->set_mogo(true);
    delay(200);

    Task abc5([&] {
        delay(200);
        intake->set_stop_condition(nullptr);
        intake->forwards();
    });

    robot->moveToPoint(40.5, -52, 2300, true, true, 80);
    Task abc2([&] {
        delay(500);
        arm->set_target(ALLIANCE_STAKE + 12);
    });
    robot->moveToPoint(22, -17, 1000, true, true);
    robot->timedMove(60, 200);
    runningAuton = false;
}

void bestautonfr::red_positive(Robot* robot) {
    bool runningAuton = true;

    robot->set_pose_mode(ODOM);

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();
    intake->arm = arm;
    intake->color_sort = true;

    Task updates([&]() {
        while (runningAuton) {
            arm->update();
            intake->update();
            delay(1);
        }
    });

    auto stage_1_stop = [&] {
        return intake->detected_ring(STAGE_2);
    };

    robot->set_pose(-58.808, -23.725, 180);
    robot->moveToPoint(-23.5, -23.5, 2000, false, false, 80);
    robot->set_mogo(true);

    delay(200);

    intake->forwards();

    robot->moveToPoint(-30.5, -51, 1000, true, false, 100);
    robot->moveToPoint(-22, -16, 2000, true, true, 60);

    arm->set_target(ALLIANCE_STAKE);
    delay(2000);
    intake->stop();

    runningAuton = false;
}

void bestautonfr::blue_positive(Robot* robot) {
    bool runningAuton = true;

    robot->set_pose_mode(ODOM);

    for (Subsystem* subsystem : robot->subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();
    intake->arm = arm;
    intake->color_sort = true;

    Task updates([&]() {
        while (runningAuton) {
            arm->update();
            intake->update();
            delay(1);
        }
    });

    auto stage_1_stop = [&] {
        return intake->detected_ring(STAGE_2);
    };

    robot->set_pose(58.808, -23.725, 0);
    robot->moveToPoint(23.5, -23.5, 2000, false, false, 80);
    robot->set_mogo(true);

    delay(200);

    intake->forwards();

    robot->moveToPoint(32.5, -46, 1000, true, false, 100);
    robot->moveToPoint(22, -16, 2000, true, true, 60);

    arm->set_target(ALLIANCE_STAKE);
    delay(2000);
    intake->stop();

    runningAuton = false;
}

void bestautonfr::skills(Robot* robot) {
    // robot->set_pose(0, 0, 90);
    // robot->poseSet = true;
    // robot->calibrate();
    // robot->set_pose_mode(ODOM);  

    // robot->ramsete({{0, 0}, {0, 30}, {30, 0}, {30, 30}}, fast);
    // return;
    bool runningAuton = true;

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
        while (runningAuton) {
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

    runningAuton = false;

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