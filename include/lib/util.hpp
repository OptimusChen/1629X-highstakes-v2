#pragma once

#include <math.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <functional>
#include "lib/bezier.h"

#define METERS 0.0254

namespace util {
    float degrees(float radians);
    float radians(float degrees);
    float clamp(float val, float min, float max);
    float calculate_shortest_angle(float target, float current); 
    float get_angle_to_target(float robotX, float robotY, float targetX, float targetY);
    float no_big_angles_pls(float thetaInRadiansNoDegreesLOL);
    int sign(double value);
    float cheap_norm_pdf(const float x);
    float avg(std::vector<float> values);
    void delay(int ms, std::function<void()> callback);
}