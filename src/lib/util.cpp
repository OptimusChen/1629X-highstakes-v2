#include "lib/util.hpp"

#include <iostream>
#include <filesystem>
#include <fstream>

#include "pros/rtos.hpp"

namespace util {
    float degrees(float radians) {
        return radians*(180/M_PI);
    }

    float radians(float degrees) {
        return degrees*(M_PI/180);
    }

    float clamp(float val, float min, float max) {
        return std::max(std::min(val, max), min);
    }

    float calculate_shortest_angle(float target, float current) {
        // Determine the larger and smaller angles
        float b = std::max(target, current);
        float s = std::min(target, current);

        // Calculate the difference
        float diff = b - s;

        // Calculate the shortest angular difference
        float shortest_diff = (diff <= 180) ? diff : (360 - b) + s;

        // Directional multiplier function (determines whether to turn clockwise or counterclockwise)
        auto dir_to_spin = [](float target, float current) {
            // If the angular difference is less than 180 degrees, turn clockwise, otherwise counterclockwise
            return (fmod((target - current + 360), 360) < 180) ? 1 : -1;
        };

        // Apply the directional multiplier
        return shortest_diff * dir_to_spin(target, current);
    }  

    float get_angle_to_target(float robotX, float robotY, float targetX, float targetY) {
        float deltaX = targetX - robotX;
        float deltaY = targetY - robotY;

        return fmod(degrees(atan2(deltaY, deltaX)), 360);
    }

    float no_big_angles_pls(float theta) {
        return theta - 2*M_PI*floor((theta+M_PI)/(2*M_PI));
    }

    int sign(double value) {
        return (value > 0) - (value < 0);
    }

    float cheap_norm_pdf(const float x) {
        // Approximation of the standard normal PDF
        // Coefficients for the rational approximation
        const float a = 0.3989422804014337;
        const float e = 0.59422804014337;

        // Compute the approximate normal PDF using a rational polynomial
        const float pdfApprox = a / (1.0 + e * x * x * x * x);

        return pdfApprox;
    }

    float avg(std::vector<float> values) {
        float sum = 0;
        for (float value : values) { sum += value; }
        return sum / values.size();
    }

    void delay(int ms, std::function<void()> callback) {
        pros::Task updates([&]() {
            pros::delay(ms);
            callback();
        });
    }
}