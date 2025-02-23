/**
 * Define logging library
 */
#pragma once

/**
 * Define the log type
 */
enum LogType {
    POSITION_EXPECTED,
    POSITION_REAL
};

/**
 * Define the log element
 */
struct LogElement {
    // The type of the log
    LogType type;
    // The x position
    float x;
    // The y position
    float y;
    // The heading
    float heading;
    // The log time
    float time;
};

namespace logging {
    void push_log(LogType type, float x, float y, float heading, float time);
    void dump(); 
}