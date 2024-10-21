#include "lib/odometry/trackingWheel.hpp"
#include <math.h>

#include <sstream>

using namespace lib;

TrackingWheel::TrackingWheel(Rotation* rotation, float offset, float diameter) {
    this->rotation = rotation;
    this->offset = offset;
    this->diameter = diameter;

    this->reset();
}

void TrackingWheel::reset() {
    this->rotation->reset_position();
}

float TrackingWheel::get_distance() {
    return float(this->rotation->get_position()) * this->diameter * M_PI / 36000.0f;
}