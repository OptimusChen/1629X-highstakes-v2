#include "lib/ramsete.hpp"
#include "lib/util.hpp"
#include <stdlib.h>
#include <math.h>

#define METERS 0.0245

RamseteController::RamseteController(float b, float zeta, float max) {
    this->b = b;
    this->zeta = zeta;
    this->max = max;
}

std::pair<float, float> RamseteController::calculate(float currentX, float currentY, float currentTheta, 
                              float targetX, float targetY, float targetT, 
                              float vd, float wd) {
    // Global error
    float global_error[3] = {
        targetX - currentX,
        targetY - currentY,
        targetT - currentTheta
    };

    float cos_theta = std::cos(currentTheta);
    float sin_theta = std::sin(currentTheta);

    // Transformation matrix (local error in robot's frame)
    float local_error[3] = {
        cos_theta * global_error[0] + sin_theta * global_error[1],
        -sin_theta * global_error[0] + cos_theta * global_error[1],
        global_error[2]
    };

    float ex = local_error[0];
    float ey = local_error[1];
    float et = util::no_big_angles_pls(local_error[2]);

    // Gain factor (unit: 1/sec)
    float k = 2 * zeta * std::sqrt(std::pow(wd, 2) + b * std::pow(vd, 2));

    // Linear velocity (unit: m/s)
    float v = vd * std::cos(et) + k * ex;

    // Angular velocity (unit: rad/s)
    float w = wd + k * et + (b * vd * std::sin(et) * ey) / et;

    w = w * ((METERS * 10.8) / 2);

    // Calculate wheel velocities (left, right)
    float left = (v - w) / this->max;
    float right = (v + w) / this->max;

    return {127 * left, 127 * right};
}