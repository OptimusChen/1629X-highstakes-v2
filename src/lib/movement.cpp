#include "lib/robot.hpp"
#include "lib/util.hpp"
#include "lib/motionProfiling.hpp"
#include "lib/controller/ramsete.hpp"
#include "lib/controller/feedForward.hpp"

#include "lib/logging.hpp"

using namespace lib;

#define METERS 0.0254

const float LOOP_PERIOD_MS = 10;

/**
 * Ramsete controller for autonomous motion profiling
 * @param waypoints The waypoints to follow
 * @param constraint The motion profile constraints
 * @param direction The direction to move in
 */
void Robot::ramsete(std::vector<bezier::Point> waypoints, MPConstraint constraint, Direction direction) {
	const double delta_d = 0.01;
    float trackWidthMeters = this->trackWidth * METERS;
    bool backwards = direction == Direction::BACKWARDS;

    // Convert to meters if the constraint is in inches
    if (constraint.unit == INCH) {
        constraint.speed = constraint.speed * METERS;
        constraint.accel = constraint.accel * METERS;
    }

	// Test Motion Profile
	Constraints constraints(constraint.speed, constraint.accel, this->friction_coef, constraint.accel, 2*constraint.accel, trackWidthMeters);
	ProfileGenerator profileGenerator(constraints, delta_d);

    std::vector<bezier::Point> waypointsMeters;

    for (auto waypoint : waypoints) {
        waypointsMeters.push_back({waypoint.x * METERS, waypoint.y * METERS});
    }

    bezier::BezierSpline<3> bezierPath(waypointsMeters);

    profileGenerator.generateProfile(bezierPath);

    RamseteController controller(2, 0.7);

    FeedforwardController ff(900, 130, 12);
    // FeedforwardController ff(900, 110, 10);
    PID pLoop(200, 0, 0);

    auto path = profileGenerator.getProfile();

    int path_index = 0;

    float llv = 0;
    float lrv = 0;

    int start_time = pros::millis();

    std::vector<float> velocities;

    while (true) {
        auto loop_start = pros::millis();

        float current_time = (loop_start - start_time) / 1000.0; // Convert to seconds

        if (current_time > path.back().t) break;

        // Find the two points surrounding the current time
        ProfilePoint point1(0, 0);
        ProfilePoint point2(0, 0);
        for (size_t i = 0; i < path.size() - 1; ++i) {
            if (path[i].t <= current_time && path[i + 1].t > current_time) {
                point1 = path[i];
                point2 = path[i + 1];
                break;
            }
        }

        // Linearly interpolate to get the target point
        float alpha = (current_time - point1.t) / (point2.t - point1.t);
        ProfilePoint target(0, 0);
        target.x = point1.x + alpha * (point2.x - point1.x);
        target.y = point1.y + alpha * (point2.y - point1.y);
        target.theta = point1.theta + alpha * (point2.theta - point1.theta);
        target.vel = point1.vel + alpha * (point2.vel - point1.vel);
        target.curvature = point1.curvature + alpha * (point2.curvature - point1.curvature);

        Pose pose = get_pose();

        // Convert chassis position to meters
        float x = pose.x * METERS;
        float y = pose.y * METERS;

        float adjustedTheta = pose.theta;
        float angular = target.vel * target.curvature;

        if (backwards) {
            adjustedTheta = fmod(adjustedTheta + M_PI, 2 * M_PI);  // Adjust by π for reverse    
        }

        // Calculate left and right motor power using Ramsete controller
        auto [v, w] = controller.calculate(x, y, adjustedTheta, target.x, target.y, target.theta, target.vel, angular);

		// std::cout << x << ", " << y << ", " << adjustedTheta << ", " << v << ", " << w << ", " << (point.t) << std::endl;
 
        // convert back to inches
        v /= METERS;
        if (backwards) w = -w;

        // std::cout << x << ", " << y << ", " << adjustedTheta << ", " << point.x << ", " << point.y << ", " << point.theta << ", " << point.vel << ", " << (point.vel * point.curvature) << ", " << v << ", " << w << std::endl;

        float leftCurrentVelocity = (left->get_actual_velocity() * 2 * M_PI) / 60 * (wheelDiameter / 2);
        float rightCurrentVelocity = (right->get_actual_velocity() * 2 * M_PI) / 60 * (wheelDiameter / 2);

        if (backwards) {
            leftCurrentVelocity *= -1;
            rightCurrentVelocity *= -1;
        }

        float leftVelocity = (v - (w * trackWidth * 0.5));
        float rightVelocity = (v + (w * trackWidth * 0.5));

        float leftAcceleration = (leftVelocity - llv) / 0.01;
        float rightAcceleration = (rightVelocity - lrv) / 0.01;

        llv = leftVelocity;
        lrv = rightVelocity;

        // std::cout << leftVelocity << ", " << leftCurrentVelocity << ", " << rightVelocity << ", " << rightCurrentVelocity << std::endl;

        float leftError = leftVelocity - leftCurrentVelocity;
        float rightError = rightVelocity - rightCurrentVelocity;

        float leftPower = ff.calculate(leftVelocity, leftAcceleration) + pLoop.calculate(leftError);
        float rightPower = ff.calculate(rightVelocity, rightAcceleration) + pLoop.calculate(rightError);

        if (backwards) {
            leftPower *= -1;
            rightPower *= -1;
        }

        left->move_voltage(leftPower);
        right->move_voltage(rightPower);

        path_index++;

        pros::c::task_delay_until(&loop_start, 10);

        // x = pose.x * METERS;
        // y = pose.y * METERS;
        // std::cout << x << ", " << y << ", " << target.x << ", " << target.y << std::endl;
    }

    // Stop motors after finishing the path or timeout
    left->move(0);
    right->move(0);
}

void Robot::turnToHeading(float target_angle, int timeout, bool reversed, int minSpeed, int maxSpeed) {
    auto usedAngular = angular;
    if (useSlowAngular) {
        usedAngular = angular_slow;
    }
    usedAngular->reset();
    
    // Record the start time using std::chrono
    auto start_time = std::chrono::high_resolution_clock::now();
    uint32_t start = 0;

    int counter = 0;

    while (true) {
        start = pros::millis();

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
        if (reversed) error = util::calculate_longest_angle(current_angle, target_angle);

        if (fabs(error) < 1) {
            counter++;
        } else {
            counter = 0;
        }

        if (counter > 5) {
            break;
        }

        // Get the output from the angular PID controller
        float output = usedAngular->calculate(error);
        
        // Set the chassis motor speeds based on the PID output
        float left_speed = util::clamp(output, -maxSpeed, maxSpeed);
        float right_speed = util::clamp(-output, -maxSpeed, maxSpeed);

        // std::cout << left_speed << std::endl;

        left->move(left_speed);
        right->move(right_speed);
        
        // Optionally sleep to avoid overwhelming the control loop
        pros::c::task_delay_until(&start, 10);
    }
    
    left->move(0);
    right->move(0);

    autonLogger.push_log(LogType::POSITION_EXPECTED, {-1, -1, target_angle, -1});
    autonLogger.push_log(LogType::POSITION_REAL, {get_pose().x, get_pose().y, get_pose().get_degrees(), -1});
}

void Robot::turnToPoint(float x, float y, int timeout, bool reversed, int minSpeed, int maxSpeed) {
    Pose pose = get_pose();
    turnToHeading(util::get_angle_to_target(pose.x, pose.y, x, y), timeout, reversed, minSpeed, maxSpeed);
}

void Robot::moveToPoint(float x, float y, int timeout, bool forwards, bool turnFirst, int maxSpeed, bool noTurn, bool slowSettling) {
    auto usedAngular = angular;
    if (useSlowAngular) {
        usedAngular = angular_slow;
    }
    // Reset the PID controllers for lateral and angular control
    lateral->reset();
    usedAngular->reset();

    // Record the start time using std::chrono
    auto start_time = std::chrono::high_resolution_clock::now();
    float multiplier = forwards ? 1.0f : -1.0f;
    int stability = 0;

    uint32_t start = 0;

    bool close = false;

    while (true) {
        start = pros::millis();
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

        // Get the output from the angular (heading) PID controller
        float turnOut = usedAngular->calculate(angularError);

        if (noTurn) turnOut = 0;

        // If the robot is close to the target, stop turning
        if (dist < 5.0f) {
            turnOut = 0;
        }

        // Apply forward or reverse multiplier
        moveOut *= multiplier;

        // If turnFirst is true and angular error is large, stop forward movement
        if (turnFirst && fabs(angularError) > 10.0f) {
            moveOut = 0;
        }

        moveOut = util::clamp(moveOut, -maxSpeed, maxSpeed);
        // turnOut = util::clamp(turnOut, -maxSpeed, maxSpeed);

        // Calculate motor speeds for tank drive (left and right motor speeds)
        float left_motor_speed = moveOut + turnOut;
        float right_motor_speed = moveOut - turnOut;

        // std::cout << left_motor_speed << ", " << right_motor_speed << std::endl;

        // Clamp motor speeds to the maximum allowed speed
        const float ratio = std::max(std::fabs(left_motor_speed), std::fabs(right_motor_speed)) / maxSpeed;
        if (ratio > 1) {
            left_motor_speed /= ratio;
            right_motor_speed /= ratio;
        }

        // Set motor speeds to move the robot toward the target
        left->move(left_motor_speed);
        right->move(right_motor_speed);

        if (dist < 5.0f) {
            close = true;
            if (slowSettling) maxSpeed = 30;
        }

        if (close) maxSpeed = 30;
        
        pros::c::task_delay_until(&start, 10);
    }

    // Stop the motors when the target is reached or timeout occurs
    left->move(0);
    right->move(0);

    autonLogger.push_log(LogType::POSITION_EXPECTED, {x, y, -1, -1});
    autonLogger.push_log(LogType::POSITION_REAL, {get_pose().x, get_pose().y, get_pose().get_degrees(), -1});
}

void Robot::timedMove(int power, int time) {
    left->move(power);
    right->move(power);

    pros::delay(time);

    left->move(0);
    right->move(0);
} 

void Robot::swingToHeading(float target_angle, int timeout, int side) {
    // ?
}

void Robot::shivaan(float x, float y, int timeout, float pct, int maxSpeed) {
    auto usedAngular = angular;
    if (useSlowAngular) {
        usedAngular = angular_slow;
    }

    // Reset the PID controllers for lateral and angular control
    lateral->reset();
    usedAngular->reset();

    // Record the start time using std::chrono
    auto start_time = std::chrono::high_resolution_clock::now();
    int stability = 0;

    Pose pose = get_pose();
    Pose lastPose = pose;
    float dist = sqrt(std::pow(pose.x - x, 2) + std::pow(pose.y - y, 2));
    float beginTurn = dist * pct;
    float driven = 0;

    float multiplier = 1;

    uint32_t start = 0;

    while (true) {
        start = pros::millis();
        pose = get_pose();

        // Calculate elapsed time
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_time - start_time;

        // Break if the timeout has been reached
        if (elapsed.count() * 1000.0f >= timeout) {
            break;
        }

        driven += std::sqrt(std::pow(pose.x - lastPose.x, 2) + std::pow(pose.y - lastPose.y, 2));
        lastPose = pose;

        if (driven > beginTurn) {
            multiplier = -1;
        }

        // Calculate the difference between the target and the robot's current position
        float deltaX = x - pose.x;
        float deltaY = y - pose.y;

        // Adjust the robot's current heading based on direction
        float adjustedTheta = pose.theta;
        if (multiplier == -1) {
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

        // std::cout << signed_dist << ": " << moveOut << std::endl;

        // Get the output from the angular (heading) PID controller
        float turnOut = usedAngular->calculate(angularError);

        // If the robot is close to the target, stop turning
        if (dist < 5.0f) {
            turnOut = 0;
        }
        moveOut = util::clamp(moveOut, -maxSpeed, maxSpeed);

        moveOut *= multiplier;

        // Calculate motor speeds for tank drive (left and right motor speeds)
        float left_motor_speed = moveOut + turnOut;
        float right_motor_speed = moveOut - turnOut;
        
        if (multiplier == -1) {
            if (angularError < 90) {
                right_motor_speed = 0;
            } else {
                left_motor_speed = 0;
            }
        }

        // Clamp motor speeds to the maximum allowed speed
        const float ratio = std::max(std::fabs(left_motor_speed), std::fabs(right_motor_speed)) / maxSpeed;
        if (ratio > 1) {
            left_motor_speed /= ratio;
            right_motor_speed /= ratio;
        }

        // Set motor speeds to move the robot toward the target
        left->move(left_motor_speed);
        right->move(right_motor_speed);

        // Check if the robot is close enough to the target to stop
        if (abs(dist) < 0.5f) {
            stability++;

            if (stability > 10) break;
        } else {
            stability = 0;
        }
        pros::c::task_delay_until(&start, 10);
    }

    // Stop the motors when the target is reached or timeout occurs
    left->move(0);
    right->move(0);
}

void Robot::relative(float distance, float maxSpeed, int timeout) {
    double headingRadians = get_pose().theta;
    double startingX = get_pose().x;
    double startingY = get_pose().y;
    double deltaX = distance * sin(headingRadians);
    double deltaY = distance * cos(headingRadians);
    double newX = startingX + deltaX;
    double newY = startingY + deltaY;
    if (distance > 0) {
        moveToPoint(newX, newY, timeout, true, false, maxSpeed, true);
    }
    else if (distance < 0) {
        moveToPoint(newX, newY, timeout, false, false, maxSpeed, true);
    }
};
