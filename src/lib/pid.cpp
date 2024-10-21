#include "lib/pid.hpp"
#include <math.h>

namespace lib {
    PID::PID(float kP, float kI, float kD) : kP(kP), kI(kI), kD(kD) {}

    float PID::calculate(float error) {
        // add error to the integral
        integral += error;

        // if error is greater than winduprange, reset integral to 0
        if (fabs(error) > winduprange && winduprange != 0) {
            integral = 0;
        }

        // calculate the derivative with error and prevError
        float derivative = error - prevError;

        // set the previous error
        prevError = error;

        return error * kP + integral * kI + derivative * kD;
    }

    void PID::reset() {
        integral = 0;
        prevError = 0;
    }

    void PID::set_winduprange(float winduprange) {
        this->winduprange = winduprange;
    }
}