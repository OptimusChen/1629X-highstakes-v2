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

#define TRACK_WIDTH 11.25

static Controller master(E_CONTROLLER_MASTER);

static auto r = Rotation(-VERTICAL);
static auto r2 = Rotation(HORIZONTAL);
static auto imu = Imu(INERTIAL_PORT);

static auto pl = TrackingWheel(&r, -0.7f, 2.0f);
static auto pd = TrackingWheel(&r2, -8.0f, 2.75f);

static MotorGroup left_motor_group({-L_DRIVE_FRONT, L_DRIVE_MID, -L_DRIVE_BACK}, MotorGears::blue, MotorUnits::rotations);
static MotorGroup right_motor_group({R_DRIVE_FRONT, -R_DRIVE_MID, R_DRIVE_BACK}, MotorGears::blue, MotorUnits::rotations);

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
// static Distance back_dist(B_DISTANCE);
// static Distance front_dist(F_DISTANCE);

Robot robot(&odom, &left_motor_group, &right_motor_group, &linear, &angular, &angular_slow);

/* Lemlib */
lemlib::Drivetrain drivetrain(
    &left_motor_group, // left motor group
    &right_motor_group,// right motor group
    TRACK_WIDTH, // 10 inch track width
    lemlib::Omniwheel::NEW_325, // using new 4" omnis
    450, // drivetrain rpm is 360
    2 // horizontal drift is 2 (for now)
);

lemlib::ControllerSettings lateral_controller(
    10, // proportional gain (kP)
    0, // integral gain (kI)
    60, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in millisecond
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
    2, // proportional gain (kP)
    0, // integral gain (kI)
    10, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(HORIZONTAL);
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(VERTICAL);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 1.8);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 1.8);

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imu // inertial sensor
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
    lateral_controller, // lateral PID settings
    angular_controller, // angular PID settings
    sensors // odometry sensors
);

void initialize() {
    lcd::initialize();
    // sec::init(&robot);

    robot.add_subsystem(new Intake());
    robot.add_subsystem(new Arm());
    
    for (Subsystem* subsystem : robot.subsystems) {
        subsystem->initialize();
    }

    chassis.calibrate(true);

    Task trackingTask = Task {[&] {
		while (true) {
            auto pose = chassis.getPose();
            pros::lcd::print(0, "p: %.2f, %.2f, %.2f", pose.x, pose.y, (pose.theta)); // print the x position

			pros::delay(10);
		}
	}};
}

void disabled() {}

void competition_initialize() {}

#define AUTON true

void autonomous() {
    if (AUTON) {
        worldsautonomous::bluePos6Ladder(&chassis, &robot);
        return;
    } 

    switch (sec::auton) {
        case 0:
            worldsautonomous::red_sawp(&chassis, &robot);
            break;
        case 1:
            worldsautonomous::blue_sawp(&chassis, &robot);
            break;
        default:
            break;
    }
}

float get_rotation_degrees(Rotation rot) {
    float measure = rot.get_angle() / 100.0f;
    if (measure > 350) return 0;
    return measure;
}

bool backwerds = true;

void opcontrol() {
    Intake* intake = robot.get_subsystem<Intake>();
    Arm* arm = robot.get_subsystem<Arm>();
    
    intake->color_sort = true;
    intake->antijam = false;
    intake->toSort = false;

    intake->set_color(BLUE);

	auto mogo = ADIDigitalOut(MOGO);
    auto doinker_left = ADIDigitalOut(DOINKER_LEFT);
    auto doinker_right = ADIDigitalOut(DOINKER_RIGHT);
    bool dlActive = false;
    bool drActive = false;
    bool mogoActive = false;

    mogo.set_value(mogoActive);

    bool lbSetting = false;

    std::unordered_map<controller_digital_e_t, std::function<void()>> toggle_controls;
    std::unordered_map<controller_digital_e_t, std::pair<std::function<void(bool)>, std::function<void()>>> hold_controls;
    std::unordered_set<controller_digital_e_t> held;

    int numStates = 4;
    int states[numStates] = {DESCORE + 10, DESCORE + 20, DESCORE + 30, DESCORE + 40};
    int state = 0;

    double pct = 1.0;
    int counter = 0;
    
    const float armP = ARM_P_VALUE;

    robot.set_brake_mode(MOTOR_BRAKE_HOLD);

    arm->liftPID.kP = armP;

    bool abc = false;
    bool abc2 = false;

    hold_controls.emplace(E_CONTROLLER_DIGITAL_L1, std::make_pair(
        [&](bool firstActivation) {
            if (!lbSetting) {
                if (arm->armTarget == REST || abc) {
                    abc = true;
                    arm->set_target(LOAD);
                } else {
                    arm->move(127);
                }
            }
        },
        [&]() {
            if (!lbSetting) {
                arm->liftPID.kP = armP;
                arm->moving = false;
                if (counter < 250) {
                    if (arm->armTarget == (LOAD+1)) {
                        arm->set_target(MID + 40);
                        abc2 = false;
                    } else if (arm->armTarget == (MID + 40)) {
                        // arm->liftPID.kP = 1;
                        arm->set_target(ALLIANCE_STAKE + 10);
                    } else {
                        arm->set_target(LOAD+1);
                    }
                } else {
                    arm->set_target(LOAD+1);
                }

                if (abc) abc2 = true;
                else abc2 = false;
                abc = false;
            }
        }
    ));

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_X, [&]() {
        intake->color_sort = !intake->color_sort;
        // pct += 0.1;
        // if (pct > 1) pct = 0.0;
    });

    hold_controls.emplace(E_CONTROLLER_DIGITAL_A, std::make_pair(
        [&](bool firstActivation) {
            arm->move(-127);
            if (abs(arm->motor->get_current_draw()) > 2000) {
                arm->armpos = arm->rotation->get_angle() / 100.0f;
                arm->set_target(REST);
            }
        },
        [&]() {
            
        }
    ));

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_L1, [&]() {
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
            // arm->liftPID.kP = 1;
            if (firstActivation) {
                arm->set_target(states[0]);
            }
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

    double start = pros::millis();

    while (true) {
        std::cout << "Arm: " << arm->motor->get_actual_velocity() << std::endl;
        std::cout << "Arm 2: " << arm->motor->get_voltage() << std::endl;
        std::cout << "Arm 2: " << backwerds << std::endl;

        if (backwerds) {
            std::cout << "Voltage: " << arm->motor->get_voltage() << std::endl;
            arm->motor->move(-127);
            arm->moving = true;

            if (pros::millis() - start > 500 && abs(arm->motor->get_actual_velocity()) < 1) {
                arm->motor->brake();
                backwerds = false;
                arm->armpos = 0;
                arm->set_target(REST);
                arm->moving = false;
            }
        } 

        float rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        float leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        // lib::opcontrol::arcade(robot, rightX, leftY);
        lib::opcontrol::tank(robot, pct * master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), pct * master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

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

        master.print(0, 0, "Intake: %.2f°C", intake->hooks.get_temperature());
        pros::delay(1);
        master.print(1, 0, "Arm: %.2f°C", arm->motor->get_temperature());
        pros::delay(1);
        master.print(2, 0, "Drive: %.2f°C", averageTemperature);
        pros::delay(1);
    }
}
