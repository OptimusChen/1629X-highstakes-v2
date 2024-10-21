#pragma once

namespace lib {
    class Pose {
        public:
            float x;
            float y;
            float theta;

            Pose(float x, float y, float theta, bool radians=false);
            Pose();

            float get_degrees();
            float get_radians();
    };
}