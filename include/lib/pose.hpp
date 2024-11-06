#pragma once

#define INCH 0
#define METER 1

namespace lib {
    class Pose {
        public:
            float x;
            float y;
            float theta;

            int unit = INCH;

            Pose(float x, float y, float theta, bool radians=false);
            Pose();

            float get_degrees();
            float get_radians();

            Pose meters();
            Pose inches();
    };
}