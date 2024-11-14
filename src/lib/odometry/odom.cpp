#include "lib/odometry/odom.hpp"
#include "lib/util.hpp"
#include <math.h>

using namespace lib;

Task* tracking = nullptr;

Odom::Odom(TrackingWheel* parallel, TrackingWheel* perpendicular, Imu* inertial) {
    this->parallel = parallel;
    this->perpendicular = perpendicular;
    this->inertial = inertial;
}

Odom::Odom(int rpm, float wheelDiameter, float trackWidth, MotorGroup* leftMotors, MotorGroup* rightMotors, Imu* inertial) {
    this->rpm = rpm;
    this->wheelDiameter = wheelDiameter;
    this->trackWidth = trackWidth;
    this->inertial = inertial;
    this->left = leftMotors;
    this->right = rightMotors;

    left->tare_position_all();
    right->tare_position_all();
    // left->set_encoder_units(motor_encoder_units_e::E_MOTOR_ENCODER_COUNTS);
    // right->set_encoder_units(motor_encoder_units_e::E_MOTOR_ENCODER_ROTATIONS);
}

void Odom::set_position(float x, float y, float theta, bool radians) {
    this->x = x;
    this->y = y;
    this->theta = theta;

    if (!radians) this->theta = util::radians(theta);
}

Pose Odom::get_pose() {
    return Pose(this->x, this->y, this->theta, true);
}

float Odom::get_right_encoder_travelled() {
    std::vector<double> positions = this->right->get_position_all();
    std::vector<float> distances;
    for (int i = 0; i < positions.size(); i++) {
        float in = 600;
        distances.push_back(positions[i] * (wheelDiameter * M_PI / rpm));
    }
    return util::avg(distances);
}

float Odom::get_left_encoder_travelled() {
    std::vector<double> positions = this->left->get_position_all();
    std::vector<float> distances;
    for (int i = 0; i < positions.size(); i++) {
        float in = 600;
        distances.push_back(positions[i] * (wheelDiameter * M_PI / rpm));
    }
    return util::avg(distances);
}

void Odom::update() {
    float recordedTheta = util::radians(-inertial->get_rotation());
    bool encoders = parallel == nullptr;

    float parallelTravel = 0;
    float perpendicularTravel = 0;

    if (!encoders) {
        parallelTravel = parallel->get_distance();
        perpendicularTravel = perpendicular->get_distance();
    } else {
        parallelTravel = get_left_encoder_travelled();
    }

    // std::cout << parallelTravel << std::endl;

    // calculate the change in sensor values
    float deltaTheta = recordedTheta - prevTheta;

    // update the previous sensor values
    prevTheta = recordedTheta;

    // calculate the heading of the robot
    float heading = theta;
    
    heading += deltaTheta;

    float deltaHeading = heading - theta;
    float avgHeading = theta + deltaHeading / 2;

    // calculate change in x and y
    float deltaX = 0;
    float deltaY = 0;

    deltaX = parallelTravel - prevParallel;
    deltaY = perpendicularTravel - prevPerpendicular;
    
    prevParallel = parallelTravel;
    prevPerpendicular = perpendicularTravel;

    // calculate local x and y
    float localX = 0;
    float localY = 0;
    if (deltaHeading == 0) { // prevent divide by 0
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + (encoders ? 0 : perpendicular->offset));
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + (encoders ? -(trackWidth / 2) : parallel->offset));
    }

    // calculate global x and y
    // std::cout << localX << ", " << localY << ", " << avgHeading << std::endl;
    x += localY * sin(avgHeading);
    y += localY * -cos(avgHeading);
    x += localX * cos(avgHeading);
    y += localX * sin(avgHeading);
    theta = heading;

    theta = fmod(theta, 2*M_PI);
    
    // If the result is negative, add 2π to bring it into the [0, 2π) range
    if (theta < 0) {
        theta += 2*M_PI;
    }
}

void Odom::start() {
    if (tracking == nullptr) {
        tracking = new pros::Task {[=] {
            while (true) {
                update();
                pros::delay(10);
            }
        }};
    }
}