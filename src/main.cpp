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

#include "lib/selector.hpp"
#include "liblvgl/lvgl.h"

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
        10, // kP
        0.02, // kI
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

static Robot robot(&odom, &left_motor_group, &right_motor_group, &linear, &angular);

static Angle angle() {
    float angle = fmod(robot.get_pose().theta - (M_PI / 2), (2 * M_PI));
    if (angle < 0) angle = 2 * M_PI + angle;
    
    return angle * radian;
}

static loco::DistanceSensorModel rightDistance(Eigen::Vector3f((-3.5_in).getValue(), (-3.5_in).getValue(), (270_deg).getValue()), right_dist);
static loco::DistanceSensorModel leftDistance(Eigen::Vector3f((-3.5_in).getValue(), (3.5_in).getValue(), (90_deg).getValue()), left_dist);
static loco::DistanceSensorModel backDistance(Eigen::Vector3f((0_in).getValue(), (-6.25_in).getValue(), (180_deg).getValue()), back_dist);
static loco::DistanceSensorModel frontDistance(Eigen::Vector3f((4_in).getValue(), (-5.25_in).getValue(), (0_deg).getValue()), front_dist);

static loco::ParticleFilter<PARTICLES> particleFilter(angle);

#define BLUE 0
#define RED 1
#define NONE -1

static int color = BLUE;

void initialize() {
	pros::lcd::initialize();

	robot.calibrate();
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
   	
    bestautonfr::skills(&robot);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

float get_rotation_degrees(Rotation rot) {
    float measure = rot.get_angle() / 100.0f;
    if (measure > 350) return 0;
    return measure;
}

void opcontrol() {
    for (Subsystem* subsystem : robot.subsystems) {
        subsystem->initialize();
    }

    Intake* intake = robot.get_subsystem<Intake> ();
    Arm* arm = robot.get_subsystem<Arm>();
    
    intake->arm = arm;
    intake->color_sort = false;
    intake->antijam = false;

	auto mogo = ADIDigitalOut(MOGO);
    auto corner_arm = ADIDigitalOut(DOINKER);
    bool mogoActive = false;
    bool cornerActive = false;

    std::unordered_map<controller_digital_e_t, std::function<void()>> toggle_controls;
    std::unordered_map<controller_digital_e_t, std::pair<std::function<void(bool)>, std::function<void()>>> hold_controls;
    std::unordered_set<controller_digital_e_t> held;

    hold_controls.emplace(E_CONTROLLER_DIGITAL_L1, std::make_pair(
        [&](bool firstActivation) {
            arm->move(127);
        },
        [&]() {
            arm->set_target(LOAD);
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_L2, std::make_pair(
        [&](bool firstActivation) {
            arm->set_target(REST);
        }, 
        [&]() {
            arm->set_target(REST);
        }
    ));

    // arm end

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_RIGHT, [&]() {
        mogoActive = !mogoActive;
        mogo.set_value(mogoActive);
    });

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_Y, [&]() {
        cornerActive = !cornerActive;
        corner_arm.set_value(cornerActive);
    });

    hold_controls.emplace(E_CONTROLLER_DIGITAL_R1, std::make_pair(
        [&](bool firstActivation) {
            intake->hooks.move(127);
        },
        [&]() {
            intake->stop();
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_R2, std::make_pair(
        [&](bool firstActivation) {
            intake->hooks.move(-127);
        },
        [&]() {
            intake->stop();
        }
    ));

    while (true) {
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        lib::opcontrol::arcade(robot, leftY, rightX);

        for (Subsystem* subsystem : robot.subsystems) {
            subsystem->update();
        }

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

        master.print(0, 0, "Intake: %.0f°C", motor_get_temperature(HOOKS));
        // delay(1);
        // master.print(0, 0, "Avg: %.0f°C", averageTemperature);

        pros::delay(1);
    }
}
