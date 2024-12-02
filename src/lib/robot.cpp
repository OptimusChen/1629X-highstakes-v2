#include "lib/robot.hpp"
#include "lib/util.hpp"
#include "lib/controller/pid.hpp"
#include "lib/controller/feedForward.hpp"
#include "lib/bezier.h"
#include "lib/motionProfiling.hpp"
#include "lib/controller/ramsete.hpp"
#include "controls.hpp"
#include <math.h>
#include <iomanip>

using namespace lib;

ADIDigitalOut arm_left(ARM_PISTON_LEFT);
ADIDigitalOut arm_right(ARM_PISTON_RIGHT);
ADIDigitalOut mogo_left(MOGO_LEFT);
ADIDigitalOut mogo_right(MOGO_RIGHT);

Motor hooks(HOOKS);

Robot::Robot(Odom* odom, MotorGroup* left, MotorGroup* right, PID* lateral, PID* angular) {
    this->odometry = odom;

    this->left = left;
    this->right = right;

    this->lateral = lateral;
    this->angular = angular;
}

void Robot::set_pose(float x, float y, float theta, bool radians) {
    odometry->set_position(x, y, theta, radians);
}

void Robot::set_pose_mode(int mode) {
    this->poseMode = mode;
}

Pose Robot::get_pose() {
    if (this->poseMode == MCL) {
        auto pred = particleFilter->getPrediction();
        float cartesianX = -pred.y() * metre.Convert(inch);
        float cartesianY = pred.x() * metre.Convert(inch);
        return Pose(cartesianX, cartesianY, odometry->get_pose().theta, true);
    }
    return odometry->get_pose();
}

void Robot::calibrate() {
    odometry->inertial->reset(true);

    odometry->start();
}

void Robot::set_constants(float wheelDiameter, int rpm, float mass, float trackWidth, float friction_coef) {
    this->wheelDiameter = wheelDiameter;
    this->rpm = rpm;
    this->mass = mass;
    this->trackWidth = trackWidth;
    this->friction_coef = friction_coef;
}

void Robot::turnToHeading(float target_angle, int timeout) {
    angular->reset();
    
    // Record the start time using std::chrono
    auto start_time = std::chrono::high_resolution_clock::now();
    
    while (true) {
        Pose pose = get_pose();

        // Calculate elapsed time
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_time - start_time;
        
        // Break if the timeout has been reached
        if (elapsed.count() * 1000.0f >= timeout) {
            break;
        }

        // Get the current robot heading (converted from radians to degrees)
        float current_angle = pose.theta * 180.0f / M_PI;
        
        // Calculate the error in heading (shortest angle)
        float error = util::calculate_shortest_angle(current_angle, target_angle);
        
        // Get the output from the angular PID controller
        float output = angular->calculate(error);
        
        // Set the chassis motor speeds based on the PID output
        float left_speed = util::clamp(output, -127, 127);
        float right_speed = util::clamp(-output, -127, 127);

        std::cout << left_speed << std::endl;

        left->move(left_speed);
        right->move(right_speed);
        
        // Check if the robot is within tolerance for both angle and position
        if (fabs(error) < 0.05 && fabs(pose.theta) < 0.05) {
            break;
        }
        
        // Optionally sleep to avoid overwhelming the control loop
        pros::delay(10);
    }
    
    left->move(0);
    right->move(0);
}

    void Robot::turnToPoint(float x, float y, int timeout) {
        Pose pose = get_pose();
        turnToHeading(util::get_angle_to_target(pose.x, pose.y, x, y), timeout);
    }

void Robot::moveToPoint(float x, float y, int timeout, bool forwards, bool turnFirst) {
    // Reset the PID controllers for lateral and angular control
    lateral->reset();
    angular->reset();

    // Record the start time using std::chrono
    auto start_time = std::chrono::high_resolution_clock::now();
    float multiplier = forwards ? 1.0f : -1.0f;

    while (true) {
        Pose pose = get_pose();

        // Calculate elapsed time
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_time - start_time;

        // Break if the timeout has been reached
        if (elapsed.count() * 1000.0f >= timeout) {
            break;
        }

        // Calculate the difference between the target and the robot's current position
        float deltaX = x - pose.x;
        float deltaY = y - pose.y;

        // Adjust the robot's current heading based on direction
        float adjustedTheta = pose.theta;
        if (!forwards) {
            adjustedTheta = fmod(adjustedTheta + M_PI, 2 * M_PI);  // Adjust by π for reverse
        }

        // Calculate the Euclidean distance (magnitude) to the target
        float dist = sqrt(deltaX * deltaX + deltaY * deltaY);

        std::cout << dist << std::endl;

        // Calculate the robot's heading vector
        float robot_heading_x = cos(adjustedTheta);
        float robot_heading_y = sin(adjustedTheta);

        // Project the vector to the target onto the robot's heading
        float signed_dist = dist * ((deltaX * robot_heading_x + deltaY * robot_heading_y) / dist);

        // Get the output from the lateral (distance) PID controller
        float moveOut = lateral->calculate(signed_dist);

        // Calculate the angular error between the current and target heading
        float target_angle = fmod(util::get_angle_to_target(pose.x, pose.y, x, y), 360.0f);
        if (target_angle < 0) {
            target_angle += 360.0f;
        }

        float adj = adjustedTheta * 180.0f / M_PI;  // Convert to degrees for the calculation

        float angularError = util::calculate_shortest_angle(adj > 0 ? adj : 360.0f + adj, target_angle);

        // std::cout << pose.x << ", " << pose.y << ": " << target_angle << " - " << adj << " = " << angularError << std::endl;

        // Adjust the movement output if the angular error is significant
        moveOut *= cos(angularError * M_PI / 180.0f);  // Convert angular error to radians

        std::cout << signed_dist << ": " << moveOut << std::endl;

        // Get the output from the angular (heading) PID controller
        float turnOut = angular->calculate(angularError);

        // If the robot is close to the target, stop turning
        if (dist < 5.0f) {
            turnOut = 0;
        }

        // Apply forward or reverse multiplier
        moveOut *= multiplier;

        // If turnFirst is true and angular error is large, stop forward movement
        if (turnFirst && fabs(angularError) > 25.0f) {
            moveOut = 0;
        }

        // std::cout << moveOut << std::endl;

        // Calculate motor speeds for tank drive (left and right motor speeds)
        float left_motor_speed = moveOut + turnOut;
        float right_motor_speed = moveOut - turnOut;

        // Clamp motor speeds to the maximum allowed speed
        left_motor_speed = util::clamp(left_motor_speed, -127, 127);
        right_motor_speed = util::clamp(right_motor_speed, -127, 127);

        const float ratio = std::max(std::fabs(left_motor_speed), std::fabs(right_motor_speed)) / 127;
        if (ratio > 1) {
            left_motor_speed /= ratio;
            right_motor_speed /= ratio;
        }

        // Set motor speeds to move the robot toward the target
        left->move(left_motor_speed);
        right->move(right_motor_speed);

        // Check if the robot is close enough to the target to stop
        if (dist < 0.05f && fabs(angularError) < 0.05f) {
            break;
        }

        // Optionally, add a small delay to prevent overwhelming the control loop
        pros::delay(10);
    }

    // Stop the motors when the target is reached or timeout occurs
    left->move(0);
    right->move(0);
}

#define METERS 0.0254

const float LOOP_PERIOD_MS = 10;

/*
# Assuming you have a VEX motor object called "motor" and its wheel radius is 0.025 meters

motor_rpm = motor.get_velocity()  # Get motor RPM from encoder data

linear_velocity_m_s = (motor_rpm * 2 * math.pi) / 60 * 0.025 

print("Linear velocity:", linear_velocity_m_s, "m/s") 

*/

void Robot::ramsete(std::vector<bezier::Point> waypoints, MPConstraint constraint, bool forwards) {
	const double delta_d = 0.01;
    float trackWidthMeters = this->trackWidth * METERS;

    if (constraint.unit == INCH) {
        constraint.speed = constraint.speed * METERS;
        constraint.accel = constraint.accel * METERS;
    }

	// Test Motion Profile
	auto constraints = new Constraints(constraint.speed, constraint.accel, this->friction_coef, constraint.accel, 2*constraint.accel, trackWidthMeters);

	auto profileGenerator = new ProfileGenerator(constraints, delta_d);

    std::vector<bezier::Point> waypointsMeters;

    for (auto waypoint : waypoints) {
        waypointsMeters.push_back({waypoint.x * METERS, waypoint.y * METERS});
    }

    bezier::BezierSpline<3> bezierPath(waypointsMeters);

    profileGenerator->generateProfile(bezierPath);

    RamseteController controller(2, 0.7);

    FeedforwardController ffLeft(900, 115, 13);
    PID pLoopLeft(0, 0, 0);
    FeedforwardController ffRight(900, 105, 13);
    PID pLoopRight(0, 0, 0);

    auto path = profileGenerator->getProfile();

    int path_index = 0;

    while (true) {
        auto loop_start = pros::micros();

        if (path_index >= path.size()) break;

        // Get target point from the motion path
        ProfilePoint point = path[path_index];

        Pose pose = get_pose();

        // Convert chassis position to meters
        float x = pose.x * METERS;
        float y = pose.y * METERS;

        float adjustedTheta = pose.theta;

        if (!forwards) {
            adjustedTheta = fmod(adjustedTheta + M_PI, 2 * M_PI);  // Adjust by π for reverse            
        }

        // Calculate left and right motor power using Ramsete controller
        auto [v, w] = controller.calculate(x, y, adjustedTheta, point.x, point.y, point.theta, point.vel, point.vel * point.curvature);

		// std::cout << x << ", " << y << ", " << adjustedTheta << ", " << v << ", " << w << ", " << (point.t) << std::endl;
 
        // convert back to inches
        v /= METERS;

        if (!forwards) w = -w;

        float leftCurrentVelocity = (left->get_actual_velocity() * 2 * M_PI) / 60 * (wheelDiameter / 2);
        float rightCurrentVelocity = (right->get_actual_velocity() * 2 * M_PI) / 60 * (wheelDiameter / 2);

        float leftVelocity = (v - (w * trackWidth * 0.5));
        float rightVelocity = (v + (w * trackWidth * 0.5));

        float acceleration = point.accel / METERS;
        float angularAcceleration = acceleration * point.curvature;

        std::cout << leftVelocity << ", " << leftCurrentVelocity << ", " << rightVelocity << ", " << rightCurrentVelocity << std::endl;

        float leftError = leftVelocity - leftCurrentVelocity;
        float rightError = rightVelocity - rightCurrentVelocity;

        float leftPower = ffLeft.calculate(leftVelocity, acceleration - angularAcceleration) + pLoopLeft.calculate(leftError);
        float rightPower = ffRight.calculate(rightVelocity, acceleration + angularAcceleration) + pLoopRight.calculate(rightError);

        if (!forwards) {
            leftPower *= -1;
            rightPower *= -1;
        }

        left->move_voltage(leftPower);
        right->move_voltage(rightPower);

        path_index++;

        float duration_loop = pros::micros() - loop_start;
        duration_loop /= 1000.0f;

        // Calculate the remaining time to sleep to maintain the desired loop period
        double sleep_time_ms = LOOP_PERIOD_MS - duration_loop;
        if (sleep_time_ms > 0) {
            pros::delay(sleep_time_ms);
        }
    }

    // Stop motors after finishing the path or timeout
    left->move(0);
    right->move(0);
}

void Robot::set_pf(loco::ParticleFilter<PARTICLES>* particleFilter) {
    this->particleFilter = particleFilter;
}

void Robot::set_arm_pistons(bool value) {
    arm_left.set_value(value);
    arm_right.set_value(value);
}

void Robot::set_mogo(bool value) {
    mogo_left.set_value(value);
    mogo_left.set_value(value);
}

void Robot::intake(bool reverse) {
    hooks.move(reverse ? -127 : 127);
}