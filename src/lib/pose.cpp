#include "lib/pose.hpp"
#include "lib/util.hpp"

using namespace lib;

Pose::Pose(float x, float y, float theta, bool radians) {
    this->x = x;
    this->y = y;
    this->theta = theta;

    if (!radians) this->theta = util::radians(theta);
}

float Pose::get_degrees() {
    return util::degrees(this->theta);
}

float Pose::get_radians() {
    return this->theta;
}

Pose Pose::meters() {
    if (unit == METER) return *this;

    unit = METER;

    return Pose(x * METERS, y * METERS, theta, true);
}

Pose Pose::inches() {
    if (unit == INCH) return *this;

    unit = INCH;

    return Pose(x / METERS, y / METERS, theta, true);
}