#include "lib/robot.hpp"
#include "lib/util.hpp"
#include "lib/controller/pid.hpp"
#include "lib/bezier.h"
#include "lib/motionProfiling.hpp"
#include "lib/controller/ramsete.hpp"
#include "lib/units.hpp"
#include <math.h>
#include <iomanip>

using namespace lib;

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

Pose Robot::get_pose() {
    return odometry->get_pose();
}

void Robot::calibrate() {
    odometry->inertial->reset(true);

    odometry->start();
}

// void Robot::set_velocityController(VelocityController vel) {
//     this->vel = vel;
// }

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

void Robot::ramsete(std::vector<bezier::Point> waypoints, bool forwards) {
    float max_speed = ((this->rpm / 60.0f) * (M_PI * this->wheelDiameter * METERS));

    float trackWidthMeters = this->trackWidth * METERS;
    float motorConst = 0.175;

    // NOTE: ASSUMES 6 MOTOR DRIVE ON BLUE CARTS
	double force = motorConst / ((this->wheelDiameter * METERS) / 2);
	double accel = (force * 6) / this->mass;

	double jerk = accel * 2;

	const double delta_d = 0.01;

	// Test Motion Profile
	auto constraints = new Constraints(max_speed, accel, this->friction_coef, accel, jerk, trackWidthMeters);

	auto profileGenerator = new ProfileGenerator(constraints, delta_d);

    bezier::Bezier<3> bezierPath(waypoints);

    profileGenerator->generateProfile(bezierPath);

    RamseteController controller(2, 0.7);
    // VelocityController lateral_vel();
    // VelocityController angular_vel();

    auto path = profileGenerator->getProfile();

    size_t path_index = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    // Target loop time (10 ms)
    float LOOP_PERIOD_MS = 10;

    bool running = true;

    while (running) {
        auto loop_start = std::chrono::high_resolution_clock::now();

        // Check timeout and end condition
        auto current_time = std::chrono::high_resolution_clock::now();
        if (path_index >= path.size()) {
            running = false;
            break;
        }

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

        if (!forwards) w = -w;

        float leftVelocity = (v - w * trackWidthMeters * 0.5);
        float rightVelocity = (v + w * trackWidthMeters * 0.5);

        double leftPower = leftVelocity * 120/max_speed;
        double rightPower = rightVelocity * 120/max_speed;

        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / (120);
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        if (!forwards) {
            leftPower *= -1;
            rightPower *= -1;
        }

        // std::cout << "left power: " << leftPower << " right power: " << rightPower << std::endl;

        left->move(leftPower);
        right->move(rightPower);
        
        auto loop_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> loop_duration = loop_end - loop_start;

        // Calculate the remaining time to sleep to maintain the desired loop period
        int sleep_time_ms = LOOP_PERIOD_MS - loop_duration.count();
        if (sleep_time_ms > 0) {
            pros::delay(sleep_time_ms);
        }

        // std::cout << "left real: " << left->get_voltage() / 100 << " right real: " << right->get_voltage() / 100 << std::endl;
        path_index++;
    }

    // Stop motors after finishing the path or timeout
    left->move(0);
    right->move(0);
}