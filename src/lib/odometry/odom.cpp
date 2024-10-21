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

void Odom::set_position(float x, float y, float theta, bool radians) {
    this->x = x;
    this->y = y;
    this->theta = theta;

    if (!radians) this->theta = util::radians(theta);
}

Pose Odom::get_pose() {
    return Pose(this->x, this->y, this->theta, true);
}

void Odom::update() {
    float recordedTheta = util::radians(-inertial->get_rotation());
    float parallelTravel = parallel->get_distance();
    float perpendicularTravel = perpendicular->get_distance();

    // calculate the change in sensor values
    // float deltaParallel = parallelTravel - prevParallel;
    // float deltaPerpendicular = perpendicularTravel - prevPerpendicular;
    float deltaTheta = recordedTheta - prevTheta;

    // update the previous sensor values
    // prevParallel = parallelTravel;
    // prevPerpendicular = perpendicularTravel;
    prevTheta = recordedTheta;

    // calculate the heading of the robot
    float heading = theta;
    
    heading += deltaTheta;

    float deltaHeading = heading - theta;
    float avgHeading = theta + deltaHeading / 2;

    // calculate change in x and y
    float deltaX = 0;
    float deltaY = 0;

    // parallelTravel = parallel->get_distance();
    // perpendicularTravel = perpendicular->get_distance();
    
    deltaY = parallelTravel - prevParallel;
    deltaX = perpendicularTravel - prevPerpendicular;
    
    prevParallel = parallelTravel;
    prevPerpendicular = perpendicularTravel;

    // calculate local x and y
    float localX = 0;
    float localY = 0;
    if (deltaHeading == 0) { // prevent divide by 0
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + perpendicular->offset);
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + parallel->offset);
    }

    // calculate global x and y
    // std::cout << localX << ", " << localY << ", " << avgHeading << std::endl;
    x += localY * sin(avgHeading);
    y += localY * cos(avgHeading);
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