#pragma once

#include <stdlib.h>
#include <utility>

namespace lib {
    class RamseteController {
        public:
            float b;
            float zeta;

            RamseteController(float b=2, float zeta=0.7);

            std::pair <float, float> calculate(float currentX, float currentY, float currentTheta, 
                                        float targetX, float targetY, float targetT, 
                                        float vd, float wd);
    };
}