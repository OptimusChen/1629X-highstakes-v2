#include "lib/opcontrol.hpp"
#include "lib/util.hpp"
#include <cmath>

// bad system ik
#define DEADZONE 0
#define MINIMUM 0
#define CURVE 1

#define DEADZONE_T 0
#define MINIMUM_T 0
#define CURVE_T 1.01

#define CD_TURN_NONLINEARITY 0.65
#define CD_NEG_INERTIA_SCALAR 4.0
#define DRIVE_SLEW 0.02f
#define CD_SENSITIVITY 1.0

constexpr int max_voltage = 12000;

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
        // left = std::round(curve(left, DEADZONE, CURVE, MINIMUM));
        // right = std::round(curve(right, DEADZONE_T, CURVE_T, MINIMUM_T));
        
        instance.left->move(left);
        instance.right->move(right);
    }

    static double _turnRemapping(double iturn) {
        double denominator = sin(M_PI / 2 * CD_TURN_NONLINEARITY);
        double firstRemapIteration =
            sin(M_PI / 2 * CD_TURN_NONLINEARITY * iturn) / denominator;
        return sin(M_PI / 2 * CD_TURN_NONLINEARITY * firstRemapIteration) / denominator;
    }

    // On each iteration of the drive controller (where we aren't point turning) we
    // constrain the accumulators to the range [-1, 1].
    double quickStopAccumlator = 0.0;
    double negInertiaAccumlator = 0.0;
    static void _updateAccumulators() {
        if (negInertiaAccumlator > 1) {
            negInertiaAccumlator -= 1;
        } else if (negInertiaAccumlator < -1) {
            negInertiaAccumlator += 1;
        } else {
            negInertiaAccumlator = 0;
        }

        if (quickStopAccumlator > 1) {
            quickStopAccumlator -= 1;
        } else if (quickStopAccumlator < -1) {
            quickStopAccumlator += 1;
        } else {
            quickStopAccumlator = 0.0;
        }
    }

    double prevTurn = 0.0;
    double prevThrottle = 0.0;
    std::pair<double, double> cheesyDrive(double ithrottle, double iturn) {
        bool turnInPlace = false;
        double linearCmd = ithrottle;
        if (fabs(ithrottle) < DEADZONE && fabs(iturn) > DEADZONE) {
            // The controller joysticks can output values near zero when they are
            // not actually pressed. In the case of small inputs like this, we
            // override the throttle value to 0.
            linearCmd = 0.0;
            turnInPlace = true;
        } else if (ithrottle - prevThrottle > DRIVE_SLEW) {
            linearCmd = prevThrottle + DRIVE_SLEW;
        } else if (ithrottle - prevThrottle < -(DRIVE_SLEW * 2)) {
            // We double the drive slew rate for the reverse direction to get
            // faster stopping.
            linearCmd = prevThrottle - (DRIVE_SLEW * 2);
        }

        double remappedTurn = _turnRemapping(iturn);

        double left, right;
        if (turnInPlace) {
            // The remappedTurn value is squared when turning in place. This
            // provides even more fine control over small speed values.
            left = remappedTurn * std::abs(remappedTurn);
            right = -remappedTurn * std::abs(remappedTurn);

        } else {
            double negInertiaPower = (iturn - prevTurn) * CD_NEG_INERTIA_SCALAR;
            negInertiaAccumlator += negInertiaPower;

            double angularCmd =
                abs(linearCmd) *  // the more linear vel, the faster we turn
                    (remappedTurn + negInertiaAccumlator) *
                    CD_SENSITIVITY -  // we can scale down the turning amount by a
                                    // constant
                quickStopAccumlator;

            right = left = linearCmd;
            left += angularCmd;
            right -= angularCmd;

            _updateAccumulators();
        }

        prevTurn = iturn;
        prevThrottle = ithrottle;
        
        return std::make_pair(left, right);
    }

    void cheeze(Robot instance, float throttle, float turn, float threshold) {
        auto [left, right] = cheesyDrive(throttle, turn);
        // std::cout << left << " " << right << std::endl;
        instance.left->move(left*127);
        instance.right->move(right*127);
        return;
        // if(std::fabs(throttle) <= threshold){
        //     arcade(instance, turn * 127, 0);
        //     return;
        // }

        // throttle = std::fabs(throttle) > threshold ? throttle : 0;
        // // turn = std::fabs(turn) > throttle ? turn : 0;

        // double left = throttle + std::fabs(throttle) * turn;
        // double right = throttle - std::fabs(throttle) * turn;

        // double mag = std::max(std::fabs(left), std::fabs(right));
        // if(mag > 1.0){
        //     left /= mag;
        //     right /= mag;
        // }

        // instance.left->move_voltage(left * max_voltage);
        // instance.right->move_voltage(right * max_voltage);
    }
}