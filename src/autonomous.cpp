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

void bestautonfr::casey(Robot* robot) {
    bool runningAuton = true;

    robot->set_pose_mode(MCL);

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

    robot->moveToPoint(-49, 0, 500, true, false, fast_speed);

    robot->turnToHeading(270, 500);

    // robot->moveToPoint(-46.5, 10, 500, false, false, 127, true);
    robot->moveToPoint(-46.5, 21, 1000, false, false, 60);

    robot->set_mogo(true);
    delay(200);
    //-----------SCORE ONE RING------------------//

    robot->turnToHeading(0, 500);

    intake->forwards();

    robot->moveToPoint(-26, 23, 700, true, false, 127);

    //-----------OBTAIN 2nd RING------------------//

    Task lbdelay1([&] {
        delay(1300);
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
    arm->liftPID.kP = 0.5;
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

    intake->backwards();
    Task intakedelay0([&] {
        delay(400);
        intake->forwards(80);
    });

    ///////////////////////////////////////////////
    //--------------2ND SECTION----------------//
    ///////////////////////////////////////////////


    //Get ring for alliance stake

    intake->antijam = true;
    arm->set_target(LOAD);
    robot->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    // robot->moveToPoint(38, 46, 1700, true, true, 100);
    robot->moveToPoint(42, 46, 2000, true, true, 127);
    robot->set_rush_arm_right(true);

    delay(200);

    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);
    
    //Clamp second goal

    robot->turnToHeading(130, 500, false, 0, 127);
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
    
    arm->set_target(MID + 20);
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
    robot->turnToHeading(0, 750, false, 15, 127);
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
    robot->moveToPoint(40, robot->get_pose().y, 500, false, false, 80);
    arm->set_target(REST);

    ///////////////////////////////////////////////
    //--------------3RD SECTION----------------//
    ///////////////////////////////////////////////

    //score one ring
    robot->moveToPoint(48, robot->get_pose().y, 500, true, false, fast_speed);
    robot->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    robot->turnToPoint(24, 24, 500);
    robot->moveToPoint(24, 24, 1500, true, true, fast_speed);

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
    intake->forwards(30);
    Task intakedelay1([&] {
        delay(500);
        intake->forwards();
    });
    // SCORE 3 RINGS
    robot->moveToPoint(-35, -35, 1000, true, true, mid_speed - 30);
    robot->moveToPoint(-41, -41, 1000, true, true, mid_speed - 15);
    robot->turnToHeading(180, 500);
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);
    robot->moveToPoint(-58, -47, 1000, true, false, mid_speed);
    delay(250);
    robot->moveToPoint(-40, -60, 1500, true, true, mid_speed);

    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);
    // delay(500);

    //-----------Place in corner------------------//
    robot->moveToPoint(-60, -63, 1250, false, true, fast_speed);
    // robot->particleFilter->setNoiseNextUpdate(true);
    robot->set_mogo(false);
    delay(500);

    ///////////////////////////////////////////////
    //--------------4th SECTION------------------//
    ///////////////////////////////////////////////
    robot->timedMove(30, 500);
    intake->stop();
    robot->moveToPoint(-47, -40, 750, false, true, 127);
    robot->moveToPoint(-47, -21, 1250, false, true, 60);
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
    arm->set_target(MID + 40);

    //-----------LADY BROWN 1------------------//
    Task intakedelay2([&] {
        delay(500);
        intake->forwards();
    });
    robot->turnToHeading(270, 450);
    arm->liftPID.kP = 0.5;
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
    robot->moveToPoint(23, -48, 850+500, true, true, fast_speed);
    delay(250);
    robot->moveToPoint(25.5, -16, 1500, true, true, fast_speed);
    delay(250);

    robot->moveToPoint(24, -58, 1200, false, true);
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    // score 3 rings
    robot->moveToPoint(44, -47, 1000, true, false, fast_speed);
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

    // HANG!!!
    intake->hooks.brake();
    intake->stop();
    arm->set_target(SCORE);
    robot->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    robot->moveToPoint(27, -27, 1000, true);
    robot->turnToHeading(315, 750, false, 0, mid_speed);
    intake->hooks.brake();
    intake->stop();
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);
    robot->timedMove(-100, 2000);

    intake->stop();
    runningAuton = false;
    dump_all();    

}

void bestautonfr::red_rush(Robot* robot) {
    bool runningAuton = true;

    robot->set_pose_mode(MCL);

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

    // intake->forwards();
    // delay(1000000);

    robot->set_rush_arm_left(true);
    intake->antijam = false;
    intake->forwards(100);
    intake->set_stop_condition(stage_1_stop);
    robot->lateral->kP = 8;
    robot->moveToPoint(-11, 40, 1300, true, false, 127);

    robot->moveToPoint(-30.5, 19, 1250, false, true, 80);
    robot->set_mogo(true);

    robot->set_rush_arm_left(false);
    delay(500);

    intake->antijam = true;

    intake->set_stop_condition(nullptr);
    intake->forwards();

    robot->moveToPoint(-21.5, 53, 1000, true, false, 100);
    delay(500);

    robot->turnToPoint(-48, 53, 500);
    robot->set_brake_mode(E_MOTOR_BRAKE_HOLD);
    robot->moveToPoint(-48, 53, 1000, true, false, 80, false, true);
    robot->turnToHeading(135, 500);
    robot->set_brake_mode(E_MOTOR_BRAKE_COAST);

    // intake->sortNextRing = true;
    robot->timedMove(40, 1000);
    delay(500);
    // robot->timedMove(-40, 1000);

    robot->lateral->kP = 6;

    robot->moveToPoint(-52, 52, 700, false, false);
    robot->turnToHeading(270, 500);
    
    arm->set_target(LOAD);
    robot->lateral->kP = 4;
    robot->particleFilter->setAddNoise(false);
    robot->moveToPoint(-44, 23, 1250, true, true);
    arm->set_target(MID + 20);
    robot->set_lift_intake(true);
    robot->moveToPoint(-47, -4, 1000, true, true, 127);
    robot->set_lift_intake(false);
    delay(500);
    robot->set_rush_arm_right(true);
    robot->turnToHeading(184, 500, false, 0, 80);
    robot->set_rush_arm_right(false);
    delay(300);
    robot->moveToPoint(-70, 0, 500, true);
    int target = 380;
    float error = front.get_distance() - target;
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
    delay(1000);
    arm->liftPID.kP = 1.5;

    runningAuton = false;
}

void bestautonfr::blue_rush(Robot* robot) {
    bool runningAuton = true;

    robot->set_pose_mode(MCL);

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
    robot->moveToPoint(-23, -47, 1000, true, true);
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

    robot->set_pose_mode(MCL);

    Intake* intake = robot->get_subsystem<Intake>();
    Arm* arm = robot->get_subsystem<Arm>();
    intake->arm = arm;
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

    arm->set_target(ALLIANCE_STAKE + 50);
    delay(500);

    robot->moveToPoint(48, 23, 500, false, false);
    arm->set_target(REST);
    robot->moveToPoint(20, 23, 1250, false, true, 60);
    
    robot->set_mogo(true);
    delay(200);

    robot->turnToHeading(130, 500);
    intake->forwards();
    robot->moveToPoint(5, 40, 850, true, false);
    robot->moveToPoint(5, 52, 700, true, false);

    robot->moveToPoint(24, 24, 700, false, false);
    intake->set_stop_condition(stage_1_stop);
    robot->moveToPoint(24, 50, 1000, true);

    robot->turnToHeading(315, 500);
    intake->set_stop_condition(nullptr);
    intake->forwards();
    robot->set_lift_intake(true);
    robot->moveToPoint(40, 12, 750, true, false);
    intake->set_stop_condition(stage_1_stop);
    robot->set_mogo(false);
    robot->moveToPoint(50, 4, 1000, true, false, 60);

    robot->set_lift_intake(false);
    arm->set_target(MID + 80);
    
    robot->turnToHeading(45, 1000, false, 0, 40);
    robot->moveToPoint(20, -23, 1000, false, true, 80);
    robot->set_mogo(true);
    delay(200);
    robot->turnToHeading(270, 500);
    intake->set_stop_condition(nullptr);
    intake->forwards();
    robot->moveToPoint(23, -47, 1000, true, true);
    robot->particleFilter->setAddNoise(false);
    robot->moveToPoint(24, -20, 1000, false, false);
    robot->turnToHeading(135, 500);
    robot->timedMove(40, 500);
    intake->stop();

    delay(500);
    runningAuton = false;
    return;
}

void bestautonfr::red_positive(Robot* robot) {
    bool runningAuton = true;

    robot->set_pose_mode(ODOM);

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

void bestautonfr::skills(Robot* robot) {}

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