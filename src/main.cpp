#include "main.h"
#include "lib/pid.hpp"
#include "arm.hpp"
#include "controls.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <math.h>

#include "lib/selector.hpp"
#include "liblvgl/lvgl.h"

#include "lib/odometry/odom.hpp"
#include "lib/robot.hpp"
#include "lib/pid.hpp"

using namespace pros;
using namespace pros::c;
using namespace controls;
using namespace lib;

Controller master(E_CONTROLLER_MASTER);

auto r = Rotation(VERTICAL);
auto r2 = Rotation(-HORIZONTAL);
auto imu = Imu(INERTIAL_PORT);

auto pl = TrackingWheel(&r, 0.4f, 2.0f);
auto pd = TrackingWheel(&r2, 1.0f, 2.0f);

Odom odom(&pd, &pl, &imu);

MotorGroup left_motor_group({-L_DRIVE_FRONT, -L_DRIVE_MID, -L_DRIVE_BACK}, MotorGears::blue);
MotorGroup right_motor_group({R_DRIVE_FRONT, R_DRIVE_MID, R_DRIVE_BACK}, MotorGears::blue);

PID linear(	
        5, // kP
        0.02, // kI
        0.1 // kD
);

// turning PID
PID angular(
	1, // proportional gain (kP)
	0.001, // integral gain (kI)
	1 // derivative gain (kD)
);

Robot robot(&odom, &left_motor_group, &right_motor_group, &linear, &angular);

void initialize() {
	pros::lcd::initialize();

	robot.calibrate();

	Task trackingTask = Task {[=] {
		while (true) {	
			auto pose = robot.get_pose();

			pros::lcd::print(0, "x: %f", pose.x); // print the x position
			pros::lcd::print(1, "y: %f", pose.y); // print the y position
			pros::lcd::print(2, "heading: %f", pose.get_degrees()); // print the heading
			pros::delay(10);
		}
	}};

	robot.set_pose(0, 0, 90);
	robot.ramsete(4000);
	// robot.turnToHeading(90 ,20000);
	// robot.moveToPoint(0, 10, 4000, true, true);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	auto mogo = ADIDigitalOut(MOGO);
    auto corner_arm = ADIDigitalOut(CORNER_ARM);
    auto lift_intake = ADIDigitalOut(INTAKE_LIFT);
    bool mogoActive = true;

    motor_set_gearing(HOOKS, E_MOTOR_GEAR_BLUE);

    Motor a(ARM);
    Rotation r(ARM_ROT);

    Arm arm(&a, &r);

    std::unordered_map<controller_digital_e_t, std::function<void()>> toggle_controls;
    std::unordered_map<controller_digital_e_t, std::pair<std::function<void(bool)>, std::function<void()>>> hold_controls;
    std::unordered_set<controller_digital_e_t> held;

    mogo.set_value(mogoActive);

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_RIGHT, [&]() {
        mogoActive = !mogoActive;
        mogo.set_value(mogoActive);
    });

    hold_controls.emplace(E_CONTROLLER_DIGITAL_R1, std::make_pair(
        [&](bool firstActivation) {
            motor_move(HOOKS, 127);
        },
        [&]() {
            motor_brake(HOOKS);
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_R2, std::make_pair(
        [&](bool firstActivation) {
            motor_move(HOOKS, -127);
        },
        [&]() {
            motor_brake(HOOKS);
        }
    ));

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_L2, [&]() {
        arm.toggle_l2();
    });

    // on hold, the arm acts as a normal arm
    hold_controls.emplace(E_CONTROLLER_DIGITAL_L1, std::make_pair(
        [&](bool firstActivation) {
            if (firstActivation) return;
            arm.hold_l1();  
        },
        [&]() {
            arm.release_l1();
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_L2, std::make_pair(
        [&](bool firstActivation) {
            if (firstActivation) return;
            arm.hold_l2();
        },
        [&]() {
            arm.release_l2();
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_Y, std::make_pair(
        [&](bool firstActivation) {
            corner_arm.set_value(true);
        },
        [&]() {
            corner_arm.set_value(false);
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_B, std::make_pair(
        [&](bool firstActivation) {
            raise_intake();
        },
        [&]() {
            lower_intake();
        }
    ));

    while (true) {
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
		// soon tm
        // auton::arcade(leftY, rightX);

        // if arm PID is enabled recalculate the error and set voltage based off PID output
        arm.tick(held);

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

        master.print(0, 0, "Battery: %.0f%c", battery, 37);
        // pros::delay(0.1);
        // master.print(1, 0, "Temps: %.0f°C", averageTemperature);
        // pros::delay(0.1);
        // master.print(2, 0, "Port %d: %.0f°C", hotspotPort, hotspot);

        pros::delay(1);
    }
}