#include "main.h"
#include "controls.hpp"
#include "lib/bezier.h"
#include "lib/opcontrol.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.h"

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
        5.5, // kP
        0, // kI
        0.2 // kD
);

// turning PID
static PID angular(
	1, // proportional gain (kP)
	0.001, // integral gain (kI)
	1 // derivative gain (kD)
);

static Distance left_dist(L_DISTANCE);
static Distance right_dist(R_DISTANCE);
static Distance back_dist(B_DISTANCE);
static Distance front_dist(F_DISTANCE);

Robot robot(&odom, &left_motor_group, &right_motor_group, &linear, &angular);

static Angle angle() {
    float angle = fmod(robot.get_pose().theta - (M_PI / 2), (2 * M_PI));
    if (angle < 0) angle = 2 * M_PI + angle;
    
    return angle * radian;
}

static loco::DistanceSensorModel rightDistance(Eigen::Vector3f((3_in).getValue(), (-5.5_in).getValue(), (270_deg).getValue()), right_dist);
static loco::DistanceSensorModel leftDistance(Eigen::Vector3f((3_in).getValue(), (5.5_in).getValue(), (90_deg).getValue()), left_dist);
static loco::DistanceSensorModel backDistance(Eigen::Vector3f((1_in).getValue(), (-6_in).getValue(), (180_deg).getValue()), back_dist);
static loco::DistanceSensorModel frontDistance(Eigen::Vector3f((2.5_in).getValue(), (-3_in).getValue(), (0_deg).getValue()), front_dist);

static loco::ParticleFilter<PARTICLES> particleFilter(angle);

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
	pros::lcd::initialize();

    std::cout << &robot << std::endl;

    robot.set_constants(2.75, 450, 5.3, TRACK_WIDTH, 3);

    Task trackingTask = Task {[&] {
		while (true) {
            auto pose = robot.get_pose();

			pros::lcd::print(0, "x: %f", pose.x); // print the x position
			pros::lcd::print(1, "y: %f", pose.y); // print the y position
			pros::lcd::print(2, "heading: %f", util::degrees(pose.theta)); // print the heading
			pros::lcd::print(3, "bruh: %d", robot.poseMode); // print the headings

			pros::delay(10);
		}
	}};

    particleFilter.addSensor(&leftDistance);
    particleFilter.addSensor(&rightDistance);
    particleFilter.addSensor(&backDistance);
    particleFilter.addSensor(&frontDistance);

    robot.set_pf(&particleFilter);
    robot.add_subsystem(new Intake());
    robot.add_subsystem(new Arm());

    // left_motor_group.set_brake_mode_all(E_MOTOR_BRAKE_BRAKE);
    // right_motor_group.set_brake_mode_all(E_MOTOR_BRAKE_BRAKE);

    autonomous();

    // robot.set_pose(-58, 0, 0);
    // robot.initialize_particle_filter();
    // robot.poseSet = true;
    // robot.calibrate();
    // delay(2000);
    // bestautonfr::skills(&robot);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    bestautonfr::skills(&robot);
}

float get_rotation_degrees(Rotation rot) {
    float measure = rot.get_angle() / 100.0f;
    if (measure > 350) return 0;
    return measure;
}

void opcontrol() {
    for (Subsystem* subsystem : robot.subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot.get_subsystem<Intake>();
    Arm* arm = robot.get_subsystem<Arm>();
    
    intake->arm = arm;
    intake->color_sort = false;
    intake->antijam = false;

	auto mogo = ADIDigitalOut(MOGO);
    auto doinker_left = ADIDigitalOut(DOINKER_LEFT);
    auto doinker_right = ADIDigitalOut(DOINKER_RIGHT);
    bool dlActive = false;
    bool drActive = false;
    bool mogoActive = false;

    bool lbSetting = false;

    std::unordered_map<controller_digital_e_t, std::function<void()>> toggle_controls;
    std::unordered_map<controller_digital_e_t, std::pair<std::function<void(bool)>, std::function<void()>>> hold_controls;
    std::unordered_set<controller_digital_e_t> held;

    int numStates = 4;
    int states[numStates] = {REST, LOAD, SCORE, ALLIANCE_STAKE};
    int state = 0;

    double pct = 1.0;

    hold_controls.emplace(E_CONTROLLER_DIGITAL_L1, std::make_pair(
        [&](bool firstActivation) {
            if (!lbSetting) arm->move(127);
        },
        [&]() {
            if (!lbSetting) {
                arm->moving = false;
                arm->set_target(LOAD);
            }
        }
    ));

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_X, [&]() {
        pct += 0.1;
        if (pct > 1) pct = 0.0;
    });

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_L1, [&]() {
        if (lbSetting) {
            state++;
            if (state >= numStates) state = 0;

            arm->set_target(states[state]);
        }
    });

    hold_controls.emplace(E_CONTROLLER_DIGITAL_L2, std::make_pair(
        [&](bool firstActivation) {
            arm->set_target(REST);
        }, 
        [&]() {
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
            lbSetting = false;
            state = 0;
            arm->set_target(REST);
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_R1, std::make_pair(
        [&](bool firstActivation) {
            intake->hooks.move_velocity(600);
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

        std::cout << intake->optical.get_proximity() << std::endl;

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
        // delay(1);
        // master.print(0, 0, "Avg: %.0f°C", averageTemperature);

        pros::delay(1);
    }
}
