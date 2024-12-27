#include "lib/opcontrol.hpp"
#include "lib/util.hpp"
#include <cmath>

// bad system ik
#define DEADZONE 5
#define MINIMUM 15
#define CURVE 1.01

namespace lib::opcontrol {
    float curve(float val) {
        // return 0 if input is within deadzone
        if (fabs(val) <= DEADZONE) return 0;
        // g is the output of g(x) as defined in the Desmos graph
        const float g = fabs(val) - DEADZONE;
        // g127 is the output of g(127) as defined in the Desmos graph
        const float g127 = 127 - DEADZONE;
        // i is the output of i(x) as defined in the Desmos graph
        const float i = pow(CURVE, g - 127) * g * util::sign(val);
        // i127 is the output of i(127) as defined in the Desmos graph
        const float i127 = pow(CURVE, g127 - 127) * g127;
        return (127.0 - MINIMUM) / (127) * i * 127 / i127 + MINIMUM * util::sign(val);
    }

    void arcade(Robot instance, float turn, float forward) {
        forward = std::round(curve(forward));
        turn = std::round(curve(turn));
        
        instance.left->move(forward + turn);
        instance.right->move(forward - turn);
    }
}