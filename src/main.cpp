#include "main.h"
#include "controls.hpp"
#include "lib/bezier.h"
#include "lib/opcontrol.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.h"
#include "filesystem"

#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <math.h>
#include <iostream>

#include "lib/odometry/odom.hpp"
#include "lib/util.hpp"
#include "lib/robot.hpp"
#include "lib/subsystem.hpp"
#include "lib/controller/pid.hpp"
#include "autonomous.hpp"
#include "intake.hpp"
#include "arm.hpp"
#include "lib/logging.hpp"

#include "lemlib/api.hpp"
#include "s.hpp"

using namespace pros;
using namespace pros::c;
using namespace controls;
using namespace lib;

#define TRACK_WIDTH 11.5

static Controller master(E_CONTROLLER_MASTER);

static auto r = Rotation(-VERTICAL);
static auto r2 = Rotation(HORIZONTAL);
static auto imu = Imu(INERTIAL_PORT);

static auto pl = TrackingWheel(&r, -0.7f, 2.0f);
static auto pd = TrackingWheel(&r2, -8.0f, 2.75f);

static MotorGroup left_motor_group({L_DRIVE_FRONT, -L_DRIVE_MID, -L_DRIVE_BACK}, MotorGears::blue, MotorUnits::rotations);
static MotorGroup right_motor_group({-R_DRIVE_FRONT, R_DRIVE_MID, R_DRIVE_BACK}, MotorGears::blue, MotorUnits::rotations);

static Odom odom(450, 2.75, TRACK_WIDTH, &left_motor_group, &right_motor_group, &imu);

static PID linear(	
        8, // kP
        0, // kI
        1 // kD
);

// turning PID
static PID angular(
	2, // proportional gain (kP)
	0, // integral gain (kI)
	10 // derivative gain (kD)
);

static PID angular_slow(
	1, // proportional gain (kP)
	0, // integral gain (kI)
	1 // derivative gain (kD)
);

static Distance left_dist(L_DISTANCE);
static Distance right_dist(R_DISTANCE);
static Distance back_dist(B_DISTANCE);
static Distance front_dist(F_DISTANCE);

Robot robot(&odom, &left_motor_group, &right_motor_group, &linear, &angular, &angular_slow);

// convert the angle from the odometry
static Angle angle() {
    float angle = fmod(robot.get_pose().theta - (M_PI / 2), (2 * M_PI));
    if (angle < 0) angle = 2 * M_PI + angle;
    
    return angle * radian;
}

// initialize the distance sensors
// static loco::DistanceSensorModel rightDistance(Eigen::Vector3f((3.5_in).getValue(), (-6.25_in).getValue(), (270_deg).getValue()), right_dist);
// static loco::DistanceSensorModel leftDistance(Eigen::Vector3f((3.5_in).getValue(), (6.25_in).getValue(), (90_deg).getValue()), left_dist);
// static loco::DistanceSensorModel backDistance(Eigen::Vector3f((1.1_in).getValue(), (-6.5_in).getValue(), (180_deg).getValue()), back_dist);
// static loco::DistanceSensorModel frontDistance(Eigen::Vector3f((2.25_in).getValue(), (-4.75_in).getValue(), (0_deg).getValue()), front_dist);

static loco::DistanceSensorModel rightDistance(Eigen::Vector3f((4.25_in).getValue(), (-6.25_in).getValue(), (270_deg).getValue()), right_dist);
static loco::DistanceSensorModel leftDistance(Eigen::Vector3f((4.25_in).getValue(), (6.25_in).getValue(), (90_deg).getValue()), left_dist);
static loco::DistanceSensorModel backDistance(Eigen::Vector3f((2.75_in).getValue(), (-6.5_in).getValue(), (180_deg).getValue()), back_dist);
static loco::DistanceSensorModel frontDistance(Eigen::Vector3f((4.25_in).getValue(), (-4.75_in).getValue(), (0_deg).getValue()), front_dist);

static loco::ParticleFilter<PARTICLES> particleFilter(angle);

/* Lemlib */

lemlib::Drivetrain drivetrain(
    &left_motor_group, // left motor group
    &right_motor_group,// right motor group
    TRACK_WIDTH, // 10 inch track width
    lemlib::Omniwheel::NEW_275, // using new 4" omnis
    450, // drivetrain rpm is 360
    2 // horizontal drift is 2 (for now)
);

lemlib::ControllerSettings lateral_controller(
    5.5, // proportional gain (kP)
    0, // integral gain (kI)
    2, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
    1, // proportional gain (kP)
    0, // integral gain (kI)
    1, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imu // inertial sensor
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
    lateral_controller, // lateral PID settings
    angular_controller, // angular PID settings
    sensors // odometry sensors
);

#define BLUE 0
#define RED 1
#define NONE -1

static int color = BLUE;

// rd::Selector selector(&robot, {
//     {"Skills", -58, 0, 0, [](Robot* robot) {
//         bestautonfr::skills(robot);
//     }},
//     {"Red Rush", -52, 27, 25, [](Robot* robot) {
//         bestautonfr::rush(robot);
//     }},
//     {"Red AWP", -55, -15, 90, [](Robot* robot) {
//         bestautonfr::sawp(robot);
//     }}
// });

void initialize() {
    lcd::initialize();
    // sec::init();

    std::cout << &robot << std::endl;

    robot.set_constants(2.75, 450, 5.3, TRACK_WIDTH, 3);

    particleFilter.addSensor(&leftDistance);
    particleFilter.addSensor(&rightDistance);
    particleFilter.addSensor(&backDistance);
    particleFilter.addSensor(&frontDistance);

    robot.set_pf(&particleFilter);
    robot.add_subsystem(new Intake());
    robot.add_subsystem(new Arm());
    
    for (Subsystem* subsystem : robot.subsystems) {
        subsystem->initialize();
    }

    // SKILLS
    // robot.set_pose(-61, 0, 0);

    // 6+1 red
    // robot.set_pose(-51, 20, 20);

    // 6+1 blue
    // robot.set_pose(51, 20, 152);

    // sawp red
    robot.set_pose(-58, 14, 235);

    robot.poseSet = true;
    robot.calibrate();
    robot.initialize_particle_filter();
    // delay(2000);
    robot.set_pose_mode(MCL);

    // masterlog.push_log(LogType::POSITION_REAL, {robot.get_pose().x, robot.get_pose().y, robot.get_pose().theta, -1});

    Task trackingTask = Task {[&] {
		while (true) {
            auto pose = robot.get_pose();

            // std::cout << "x: " << pose.x << " y: " << pose.y << " theta: " << pose.theta << std::endl;
            
            distanceLogger.push_log(LogType::DISTANCE_SENSOR, {float(left_dist.get_distance()), float(right_dist.get_distance()), float(back_dist.get_distance()), float(front_dist.get_distance())});
            // distanceLogger.push_log(LogType::POSITION_REAL, {robot.get_pose().x, robot.get_pose().y, robot.get_pose().theta, -1});

			pros::lcd::print(0, "x: %f", pose.x); // print the x position
			pros::lcd::print(1, "y: %f", pose.y); // print the y position
			pros::lcd::print(2, "heading: %f", util::degrees(pose.theta)); // print the heading
			pros::lcd::print(3, "bruh: %d", robot.poseMode); // print the headings
			pros::lcd::print(4, "rand: %d", std::rand()); // print the headings

			pros::delay(10);
		}
	}};

    autonomous();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    bestautonfr::red_sawp(&robot);
    return;
    switch (sec::auton)
    {
        case 0:
            bestautonfr::blue_sawp(&robot);
            break;
        case 1:
            bestautonfr::blue_rush( &robot, &chassis);
            break;
        case 2:
            bestautonfr::blue_positive(&robot);
            break;
        case 3:
            bestautonfr::red_sawp(&robot);
            break;
        case 4:
            bestautonfr::red_rush( &robot, &chassis);
            break;
        case 5:
            bestautonfr::red_positive(&robot);
            break;
        case 6:
            break;
        case 7:
            bestautonfr::casey(&robot, &chassis);
            break;
    }
}

float get_rotation_degrees(Rotation rot) {
    float measure = rot.get_angle() / 100.0f;
    if (measure > 350) return 0;
    return measure;
}

void opcontrol() {
    Intake* intake = robot.get_subsystem<Intake>();
    Arm* arm = robot.get_subsystem<Arm>();
    
    intake->arm = arm;
    intake->color_sort = true;
    intake->antijam = false;

	auto mogo = ADIDigitalOut(MOGO);
    auto doinker_left = ADIDigitalOut(DOINKER_LEFT);
    auto doinker_right = ADIDigitalOut(DOINKER_RIGHT);
    bool dlActive = false;
    bool drActive = false;
    bool mogoActive = false;

    bool lbSetting = false;

    arm->set_target(REST);

    std::unordered_map<controller_digital_e_t, std::function<void()>> toggle_controls;
    std::unordered_map<controller_digital_e_t, std::pair<std::function<void(bool)>, std::function<void()>>> hold_controls;
    std::unordered_set<controller_digital_e_t> held;

    int numStates = 4;
    int states[numStates] = {REST, LOAD, SCORE, ALLIANCE_STAKE};
    int state = 0;

    double pct = 1.0;
    int counter = 0;
    
    const float armP = 1.0f;

    intake->color_sort = true;
    intake->set_color(RED);

    robot.set_brake_mode(MOTOR_BRAKE_HOLD);

    hold_controls.emplace(E_CONTROLLER_DIGITAL_L1, std::make_pair(
        [&](bool firstActivation) {
            if (!lbSetting) {
                if ((arm->rotation->get_angle() / 100.0f) < LOAD / 3) {
                    arm->move(60);
                } else {
                    arm->move(127);
                }
            }
        },
        [&]() {
            arm->liftPID.kP = armP;
            if (!lbSetting) {
                arm->moving = false;
                if (counter < 250) {
                    if (arm->armTarget == LOAD) {
                        arm->set_target(MID + 20);
                    } else if (arm->armTarget == (MID + 20)) {
                        arm->liftPID.kP = 0.5;
                        arm->set_target(ALLIANCE_STAKE - 10);
                    } else {
                        arm->set_target(LOAD);
                    }
                } else {
                    arm->set_target(LOAD);
                }
            }
        }
    ));

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_X, [&]() {
        intake->color_sort = !intake->color_sort;
        // pct += 0.1;
        // if (pct > 1) pct = 0.0;
    });

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_L1, [&]() {
        arm->liftPID.kP = armP;
        if (lbSetting) {
            state++;
            if (state >= numStates) state = 0;

            arm->set_target(states[state]);
        }
    });

    hold_controls.emplace(E_CONTROLLER_DIGITAL_LEFT, std::make_pair(
        [&](bool firstActivation) {
            robot.set_lift_intake(true);
        }, 
        [&]() {
            robot.set_lift_intake(false);
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_L2, std::make_pair(
        [&](bool firstActivation) {
            arm->liftPID.kP = armP;
            arm->set_target(REST);
        }, 
        [&]() {
            arm->liftPID.kP = armP;
            arm->set_target(REST);
        }
    ));

    // arm end

    hold_controls.emplace(E_CONTROLLER_DIGITAL_RIGHT, std::make_pair(
        [&](bool firstActivation) {
            mogo.set_value(false);
        }, 
        [&]() {
            mogo.set_value(true);
        }
    ));

    // toggle_controls.emplace(E_CONTROLLER_DIGITAL_RIGHT, [&]() {
    //     mogoActive = !mogoActive;
    //     mogo.set_value(mogoActive);
    // });

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_DOWN, [&]() {
        dlActive = !dlActive;
        doinker_left.set_value(dlActive);
    });

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_B, [&]() {
        drActive = !drActive;
        doinker_right.set_value(drActive);
    });

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_UP, [&]() {
        intake->color_sort = !intake->color_sort;
    });

    hold_controls.emplace(E_CONTROLLER_DIGITAL_Y, std::make_pair(
        [&](bool firstActivation) {
            lbSetting = true;
        },
        [&]() {
            arm->liftPID.kP = armP;
            lbSetting = false;
            state = 0;
            arm->set_target(REST);
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_R1, std::make_pair(
        [&](bool firstActivation) {
            intake->forwards();
        },
        [&]() {
            intake->stop();
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_R2, std::make_pair(
        [&](bool firstActivation) {
            intake->hooks.move_velocity(-600);
        },
        [&]() {
            intake->stop();
        }
    ));

    while (true) {
        float rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        float leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        // lib::opcontrol::arcade(robot, rightX, leftY);
        lib::opcontrol::tank(robot, pct * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), pct * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        for (Subsystem* subsystem : robot.subsystems) {
            subsystem->update();
        }

        // std::cout << intake->optical.get_proximity() << std::endl;

        for (auto control : toggle_controls) {
            if (master.get_digital_new_press(control.first) && !held.contains(control.first)) {
                control.second();
            }
        }

        for (auto control : hold_controls) {
            if (master.get_digital(control.first)) {
                control.second.first(!held.contains(control.first));
                held.insert(control.first);
            } else if (held.contains(control.first)) {
                control.second.second();
                held.erase(control.first);
            }
        }

        if (master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
            counter++;
        } else {
            counter = 0;
        }

        double battery = battery_get_capacity();

        auto drivetrainMotors = {L_DRIVE_FRONT, L_DRIVE_MID, L_DRIVE_BACK, R_DRIVE_FRONT, R_DRIVE_MID, R_DRIVE_BACK};

        double temperatureSum = 0.0;
        double hotspot = 0.0;
        int hotspotPort = 0;

        for (int port : drivetrainMotors) {
            double currentTemp = motor_get_temperature(port);

            temperatureSum += currentTemp;
            
            if (hotspot < currentTemp) {
                hotspot = currentTemp;
                hotspotPort = port;
            }
        }

        double averageTemperature = temperatureSum / drivetrainMotors.size();

        temperatureSum = 0.0;

        master.print(0, 0, "Intake: %.2f°C", pct);

        pros::delay(1);
    }
}
