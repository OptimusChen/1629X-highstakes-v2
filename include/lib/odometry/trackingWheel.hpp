#pragma once

#include "pros/rotation.hpp"

using namespace pros;

namespace lib {
    class TrackingWheel {
        public:
            Rotation* rotation;
            float offset;
            float diameter;

            TrackingWheel(Rotation* rotation, float offset, float diameter);

            void reset();
            float get_distance();
    };
}