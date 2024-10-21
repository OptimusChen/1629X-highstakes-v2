#pragma once

#include <stdlib.h>
#include <utility>

class RamseteController {
    public:
        float b;
        float zeta;
        float max;

        RamseteController(float b=2, float zeta=0.7, float max=69);

        std::pair <float, float> calculate(float currentX, float currentY, float currentTheta, 
                                      float targetX, float targetY, float targetT, 
                                      float vd, float wd);
};