#pragma once

namespace lib {
    class PID {
        public:
            float kP = 0;
            float kI = 0;
            float kD = 0;
            float winduprange = 0;

            float integral = 0;
            float prevError = 0;

            PID(float kP, float kI, float kD);
            float calculate(float error);
            void reset();
            void set_winduprange(float winduprange);
    };
}