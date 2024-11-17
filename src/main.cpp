#include "main.h"
#include "arm.hpp"
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
#include "lib/controller/pid.hpp"
#include "lib/controller/velocityController.hpp"
#include "autonomous.hpp"

using namespace pros;
using namespace pros::c;
using namespace controls;
using namespace lib;

Controller master(E_CONTROLLER_MASTER);

auto r = Rotation(-VERTICAL);
auto r2 = Rotation(HORIZONTAL);
auto imu = Imu(INERTIAL_PORT);

auto pl = TrackingWheel(&r, -0.7f, 2.0f);
auto pd = TrackingWheel(&r2, -8.0f, 2.75f);

MotorGroup left_motor_group({L_DRIVE_FRONT, -L_DRIVE_MID, -L_DRIVE_BACK}, MotorGears::blue, MotorUnits::rotations);
MotorGroup right_motor_group({-R_DRIVE_FRONT, R_DRIVE_MID, R_DRIVE_BACK}, MotorGears::blue, MotorUnits::rotations);

Odom odom(450, 2.75, 11.5, &left_motor_group, &right_motor_group, &imu);

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

// VelocityController vel(&robot, 0.01, 1, 0.01, 0.1);

Distance left_dist(L_DISTANCE);
Distance right_dist(R_DISTANCE);
Distance back_dist(B_DISTANCE);

constexpr float DRIVE_RATIO = 48.0/36.0; // EX: 36 tooth driving gear to 48 tooth driven gear.
constexpr double DRIVETRAIN_TUNING_SCALAR = 1; // Tuning variable to make sure distance matches
constexpr QLength WHEEL_RADIUS = 2.75_in/2.0; // Wheel radius
constexpr float DRIVE_NOISE = 0.35; // The desired amount in % of noise on the drive
constexpr Angle ANGLE_NOISE = 8_deg; // The noise on the angle that's desired

Robot robot(&odom, &left_motor_group, &right_motor_group, &linear, &angular);

Angle angle() {
    return Angle(fmod(robot.get_pose().theta + (M_PI / 2), (2 * M_PI)));

    // Invert the angle into the loco coordinate system
    const Angle angle = -imu.get_rotation() * degree;

    // Check to make sure the angle isn't nan, if it is it can cause issues in the position change calculations
    return isfinite(angle.getValue()) ? angle : 0.0;
}

loco::DistanceSensorModel rightDistance(Eigen::Vector3f((-3.75_in).getValue(), (-5.75_in).getValue(), (270_deg).getValue()), right_dist);
loco::DistanceSensorModel leftDistance(Eigen::Vector3f((-3.25_in).getValue(), (5.75_in).getValue(), (90_deg).getValue()), left_dist);
loco::DistanceSensorModel backDistance(Eigen::Vector3f((-4.375_in).getValue(), (5.25_in).getValue(), (180_deg).getValue()), back_dist);

loco::ParticleFilter<150> particleFilter(angle);
std::ranlux24_base de;

#define BLUE 0
#define RED 1
#define NONE -1

int color = RED;

QLength lastLeft, lastRight;

QLength getDistance(const pros::MotorGroup& motor) {
    QLength totalPosition = 0.0;

    for (double position : motor.get_position_all()) {
        totalPosition += position / DRIVE_RATIO * 2.0 * M_PI
           * DRIVETRAIN_TUNING_SCALAR * WHEEL_RADIUS;
    }

    return totalPosition/motor.size();
}

void initialize() {
	pros::lcd::initialize();

	robot.calibrate();
    robot.set_constants(2.75, 450, 6, 11.5, 0.2);

    robot.set_pose(0, 0, 90);

    // robot.set_pose_mode(MCL);
    // robot.ramsete({{0, 0}, {0, 0.5}, {0.5, 0}, {0.5, 0.5}, {0.5, 0.5}, {0.5, 1}, {0, 0.5}, {0, 1}}, true);
    // robot.ramsete({{0, 0}, {0, 0.5}, {0.5, 0}, {0.5, 0.5}}, true);

    Task trackingTask = Task {[&] {
		while (true) {	
            auto pose = robot.get_pose();

			pros::lcd::print(0, "x: %f", pose.x * METERS); // print the x position
			pros::lcd::print(1, "y: %f", pose.y * METERS); // print the y position
			pros::lcd::print(2, "x: %f", pose.x); // print the x position
			pros::lcd::print(3, "y: %f", pose.y); // print the y position
			pros::lcd::print(4, "heading: %f", pose.theta); // print the heading
			pros::delay(10);
		}
	}};

    particleFilter.addSensor(&leftDistance);
    particleFilter.addSensor(&rightDistance);
    particleFilter.addSensor(&backDistance);

    particleFilter.initUniform(-70_in, -70_in, 70_in, 70_in);

    pros::Task locoTask = pros::Task([&]() {
        uint32_t start_time = 0;

        // Run localization forever
        while (true) {
            // Store the start time to ensure that the time between updates remains consistent
            start_time = pros::millis();

            // Store the current distance of the drivetrain
            const QLength leftLength = getDistance(left_motor_group);
            const QLength rightLength = getDistance(right_motor_group);

            // Calculate the change from the previous position
            const QLength leftChange = leftLength - lastLeft;
            const QLength rightChange = rightLength - lastRight;
            
            // Store the current value as the last value for next frame
            lastLeft = leftLength;
            lastRight = rightLength;

            // Calculate the average movement, this is a cheaper way to get the movement of the drive at the center for
            // skid-steer based mechanics
            auto avg = (leftChange + rightChange) / 2.0;

            // Define the distributions to add noise to the sensor readings
            std::uniform_real_distribution avgDistribution(avg.getValue() - DRIVE_NOISE * avg.getValue(),
                                                        avg.getValue() + DRIVE_NOISE * avg.getValue());
            std::uniform_real_distribution angleDistribution(
                particleFilter.getAngle().getValue() - ANGLE_NOISE.getValue(),
                particleFilter.getAngle().getValue() + ANGLE_NOISE.getValue());

            particleFilter.update([&]() mutable {
                // Calculate noisy sensor readings
                const auto noisy = avgDistribution(de);
                const auto angle = angleDistribution(de);

                // Calculate the translation with the sensor readings
                return Eigen::Rotation2Df(angle) * Eigen::Vector2f({noisy, 0.0});
            });

            pros::c::task_delay_until(&start_time, 10);
        }
    });
   	
    bestautonfr::skills(robot);
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
	auto mogo_left = ADIDigitalOut(MOGO_LEFT);
	auto mogo_right = ADIDigitalOut(MOGO_RIGHT);
	auto arm_left = ADIDigitalOut(ARM_PISTON_LEFT);
	auto arm_right = ADIDigitalOut(ARM_PISTON_RIGHT);
    auto corner_arm = ADIDigitalOut(DOINKER);
    auto lift_intake = ADIDigitalOut(INTAKE_LIFT);
    bool mogoActive = true;
    bool armActive = true;

    bool loading = false;
    bool reverse_intake = false;
    bool stop_intake = false;
    bool holding_ring = false;
    bool color_sort = true;

    motor_set_gearing(HOOKS, E_MOTOR_GEAR_BLUE);

    std::unordered_map<controller_digital_e_t, std::function<void()>> toggle_controls;
    std::unordered_map<controller_digital_e_t, std::pair<std::function<void(bool)>, std::function<void()>>> hold_controls;
    std::unordered_set<controller_digital_e_t> held;

    // arm
    Motor leftArm(-LEFT_ARM, MotorGears::green);
    Motor rightArm(RIGHT_ARM, MotorGears::green);

    Optical optical(OPTICAL);
    optical.set_led_pwm(100);

    Rotation leftRotation(-LEFT_ROTATION);
    Rotation rightRotation(-RIGHT_ROTATION);

    PID leftLift(1.5, 0, 0.1);
    PID rightLift(1.5, 0, 0.1);

    leftLift.reset();
    rightLift.reset();

    leftRotation.reset();
    rightRotation.reset();
    leftRotation.reset_position();
    rightRotation.reset_position();

    float REST_LOAD = 0;
    float SCORE = 300;

    float armTarget = REST_LOAD;

    hold_controls.emplace(E_CONTROLLER_DIGITAL_L1, std::make_pair(
        [&](bool firstActivation) {
            // leftArm.move(100);
            // rightArm.move(100);
            armTarget += 1;
        },
        [&]() {
            armTarget = SCORE;
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_L2, std::make_pair(
        [&](bool firstActivation) {
            armTarget = REST_LOAD;
        }, 
        [&]() {
            armTarget = REST_LOAD;

            lift_intake.set_value(false);
        }
    ));

    // arm end

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_RIGHT, [&]() {
        mogoActive = !mogoActive;
        mogo_left.set_value(mogoActive);
        mogo_right.set_value(mogoActive);
    });

    toggle_controls.emplace(E_CONTROLLER_DIGITAL_LEFT, [&]() {
        armActive = !armActive;
        arm_left.set_value(armActive);
        arm_right.set_value(armActive);
    });

    hold_controls.emplace(E_CONTROLLER_DIGITAL_Y, std::make_pair(
        [&](bool firstActivation) {
            loading = true;
            color_sort = false;

            arm_left.set_value(false);
            arm_right.set_value(false);
        },
        [&]() {
            loading = false;
            color_sort = true;
            stop_intake = false;
            if (holding_ring) {
                armActive = true;
                armTarget = SCORE;

                lift_intake.set_value(true);
                arm_left.set_value(armActive);
                arm_right.set_value(armActive);
            }
        }
    ));

    hold_controls.emplace(E_CONTROLLER_DIGITAL_R1, std::make_pair(
        [&](bool firstActivation) {
            if (stop_intake) {
                motor_brake(HOOKS);
                return;
            }

            int power = loading ? 80 : 127;

            // std::cout << power << ", " << reverse_intake << ", " << stop_intake << std::endl;

            motor_move(HOOKS, reverse_intake ? -power : power);
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

    while (true) {
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
		// soon tm
        // auton::arcade(leftY, rightX);
        lib::opcontrol::arcade(robot, leftY, rightX);

        // if arm PID is enabled recalculate the error and set voltage based off PID output
        float leftPower = leftLift.calculate(armTarget - get_rotation_degrees(leftRotation));
        float rightPower = rightLift.calculate(armTarget - get_rotation_degrees(rightRotation));

        leftArm.move(leftPower);
        rightArm.move(rightPower);

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

        auto opticalMeasure = optical.get_rgb();
        
        // std::cout << opticalMeasure.red << ", " << opticalMeasure.green << ", " << opticalMeasure.blue << std::endl;
        // std::cout << optical.get_proximity() << std::endl;

        if (color == BLUE && opticalMeasure.red > 300.0f && !reverse_intake && color_sort) {
            // std::cout << "story" << std::endl;
            reverse_intake = true;
            Task {[&] {
                delay(100);
                reverse_intake = false;
            }};
        }
        if (color == RED && opticalMeasure.blue > 200.0f && !reverse_intake && color_sort) {
            reverse_intake = true;
            Task {[&] {
                delay(100);
                reverse_intake = false;
            }};   
        }

        if (loading) { 
            if (optical.get_proximity() > 100) {
                if (!holding_ring) {
                    motor_move(HOOKS, -50);
                    delay(50);
                    motor_brake(HOOKS);

                    stop_intake = true;
                    holding_ring = true;
                }
            } else {
                holding_ring = false;
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