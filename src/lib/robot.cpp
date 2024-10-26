#include "lib/robot.hpp"
#include "lib/util.hpp"
#include "lib/motionProfiling.hpp"
#include "lib/ramsete.hpp"
#include "lib/units.hpp"
#include <math.h>

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

#define METERS 0.0254

void Robot::ramsete(int timeout) {
    float max_speed = (450 * M_PI * 2.75 * METERS) / 60.0f;

    float trackWidth = 11 * METERS;

	double force = 0.15 / ((2.75 * METERS) / 2);
	double accel = (force * 6) / 5;

	double jerk = accel * 2;

	const double delta_d = 0.01;

	// Test Motion Profile
	auto constraints = new Constraints(max_speed, max_speed*3, 0.1, max_speed*3, jerk, trackWidth);

	auto profileGenerator = new ProfileGenerator(constraints, delta_d);

    CubicBezier* testPath;
	testPath = new CubicBezier({0, 0}, {0, 0.85}, {0.85, 0}, {0.85, 0.85});

    profileGenerator->generateProfile(testPath);

    RamseteController controller(2, 0.7, max_speed);

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

        Pose pose = get_pose();

        // Convert chassis position to meters
        float x = pose.x * METERS;
        float y = pose.y * METERS;

        // Get target point from the motion path
        ProfilePoint point = path[path_index];

        // Calculate left and right motor power using Ramsete controller
        auto [v, w] = controller.calculate(x, y, pose.theta, point.x, point.y, point.theta, point.vel, point.accel);

        float left_power = 127 * ((v - w * trackWidth * 0.5) / max_speed);
        float right_power = 127 * ((v + w * trackWidth * 0.5) / max_speed);

        std::cout << "left power: " << left_power << " right power: " << right_power << std::endl;

        left->move(left_power);
        right->move(right_power);

        // Move to the next point in the path
        path_index++;

        // Calculate the time spent in this loop
        auto loop_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> loop_duration = loop_end - loop_start;

        // Calculate the remaining time to sleep to maintain the desired loop period
        float sleep_time_ms = LOOP_PERIOD_MS - loop_duration.count();

        pros::delay(sleep_time_ms);
    }

    // Stop motors after finishing the path or timeout
    left->move(0);
    right->move(0);
}