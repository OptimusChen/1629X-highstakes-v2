#pragma once

enum LogType {
    POSITION_EXPECTED,
    POSITION_REAL
};

struct LogElement {
    LogType type;
    float x;
    float y;
    float heading;
    float time;
};

namespace logging {
    void push_log(LogType type, float x, float y, float heading, float time);
    void dump(); 
}