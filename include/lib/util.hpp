#pragma once

#include <math.h>
#include <stdlib.h>

namespace util {
    float degrees(float radians);
    float radians(float degrees);
    float clamp(float val, float min, float max);
    float calculate_shortest_angle(float target, float current); 
    float get_angle_to_target(float robotX, float robotY, float targetX, float targetY);
    float no_big_angles_pls(float thetaInRadiansNoDegreesLOL);
    int sign(double value);
}