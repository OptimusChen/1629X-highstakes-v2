#include "logging.hpp"
#include <vector>
#include "pros/rtos.hpp"
#include <iostream>

std::vector<LogElement> logs;

namespace logging {
    void push_log(LogType type, float x, float y, float heading, float time) {
        logs.push_back({type, x, y, heading, float(pros::millis())});
    }
    
    void dump() {
        // FILE file = fopen();
        for (auto log : logs) {
            switch (log.type)
            {
            case LogType::POSITION_EXPECTED:
                std::cout << "Expected: " << log.x << ", " << log.y << ", " << log.heading << std::endl;
                break;
            case LogType::POSITION_REAL:
                std::cout << "Real: " << log.x << ", " << log.y << ", " << log.heading << std::endl;
                break;
            default:
                break;
            }
            pros::delay(20);
        }

    }
}
