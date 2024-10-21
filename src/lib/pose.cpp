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