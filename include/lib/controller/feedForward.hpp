#include "../util.hpp"

namespace lib {
    class FeedforwardController {
        public:
            FeedforwardController(double kS, double kV, double kA)
                : kS(kS), kV(kV), kA(kA) {}

            // Calculate the required voltage for a given velocity and acceleration
            double calculate(double velocity, double acceleration) const {
                return kS * util::sign(velocity) + kV * velocity + kA * acceleration;
            }

        private:
            double kS; // Static friction coefficient
            double kV; // Velocity coefficient
            double kA; // Acceleration coefficient
    };
}