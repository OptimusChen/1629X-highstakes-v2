#pragma once
#include <array>
#include <string>
#include <vector>

#define MAX_LOG_ENTRIES 10000  // Fixed circular buffer size
#define MAX_LOGGERS 10  // Fixed circular buffer size
#define LOG_SIZE 4  // Fixed circular buffer size

/**
 * Define the log type
 */
enum class LogType {
    POSITION_EXPECTED,
    POSITION_REAL,
    LOOP_TIME,  // Log type for loop execution time'
    DISTANCE_SENSOR,  // Log type for distance sensor readings
    DELTA_MOVEMENT,  // Log type for delta movement
};

/**
 * Define a log element
 */
struct LogElement {
    LogType type;        // Type of the log
    uint32_t time;          // Timestamp
    float a;             // X position (or relevant data)
    float b;             // Y position (or relevant data)
    float c;       // Heading (or relevant data)
    float d;       // Heading (or relevant data)
};

std::array<Logger, MAX_LOGGERS> all_loggers;
size_t logger_count = 0;  // Total number of log entries across all loggers

/**
 * Logger class for managing log entries
 */
class Logger {
private:
    std::array<LogElement, MAX_LOG_ENTRIES> logs; // Fixed-size circular buffer
    size_t log_index = 0;  // Circular buffer index
    size_t valid_count = 0;
    std::string logger_name; // Unique name for identifying the logger instance

public:
    /** Constructor that registers the instance */
    Logger(const std::string& name);

    /** Push a log entry into the buffer */
    void push_log(LogType type, std::array<float, LOG_SIZE> data);

    /** Dump all logs from all instances to a single file */
    static void dump_all();
};
