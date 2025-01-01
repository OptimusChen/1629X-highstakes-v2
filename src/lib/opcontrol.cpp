#include "lib/opcontrol.hpp"
#include "lib/util.hpp"
#include <cmath>

// bad system ik
#define DEADZONE 5
#define MINIMUM 15
#define CURVE 1

#define DEADZONE_T 5
#define MINIMUM_T 15
#define CURVE_T 1.01

namespace lib::opcontrol {
    float curve(float val, int deadzone, float curve, int minimum) {
        // return 0 if input is within deadzone
        if (fabs(val) <= deadzone) return 0;
        // g is the output of g(x) as defined in the Desmos graph
        const float g = fabs(val) - deadzone;
        // g127 is the output of g(127) as defined in the Desmos graph
        const float g127 = 127 - deadzone;
        // i is the output of i(x) as defined in the Desmos graph
        const float i = pow(curve, g - 127) * g * util::sign(val);
        // i127 is the output of i(127) as defined in the Desmos graph
        const float i127 = pow(curve, g127 - 127) * g127;
        return (127.0 - minimum) / (127) * i * 127 / i127 + minimum * util::sign(val);
    }

    void arcade(Robot instance, float turn, float forward) {
        forward = std::round(curve(forward, DEADZONE, CURVE, MINIMUM));
        turn = std::round(curve(turn, DEADZONE_T, CURVE_T, MINIMUM_T));
        
        float l = forward + turn;
        float r = forward - turn;

        const float ratio = std::max(std::fabs(l), std::fabs(r)) / 127;
        if (ratio > 1) {
            l /= ratio;
            r /= ratio;
        }

        instance.left->move(l);
        instance.right->move(r);
    }

    void tank(Robot instance, float left, float right) {
        left = std::round(curve(left, DEADZONE, CURVE, MINIMUM));
        right = std::round(curve(right, DEADZONE_T, CURVE_T, MINIMUM_T));
        
        instance.left->move(left);
        instance.right->move(right);
    }
}