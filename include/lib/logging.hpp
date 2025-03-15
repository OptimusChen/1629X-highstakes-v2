#pragma once
#include <array>
#include <string>
#include <vector>
#include "pros/rtos.hpp"
#include <iostream>
#include <fstream>
#include <cstdio>

#define MAX_LOGGERS 4  // Fixed circular buffer size
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
    PARTICLE_POSITION, // Log type for particle filter position
    UNIQUE_PARTICLES, // Log type for unique particles  
    STANDARD_DEVIATION, // Log type for standard deviation
    DEVIATION_AND_UNIQUE, // Log type for deviation and unique particles
    DELTA_MOVEMENT_AND_LOOP_TIME,  // Log type for delta movement and loop time
    OBJECT_SIZE // Log type for object size
};

/**
 * Define a log element
 */
struct LogElement {
    LogType type;        // Type of the log
    int time;          // Timestamp
    float a;             // X position (or relevant data)
    float b;             // Y position (or relevant data)
    float c;       // Heading (or relevant data)
    float d;       // Heading (or relevant data)
};

// Collection of all logger instances
extern bool writingToFile;

/**
 * Logger class for managing log entries
 */
template <size_t MAX_LOG_ENTRIES>
class Logger {
private:
    std::array<LogElement, MAX_LOG_ENTRIES> logs; // Fixed-size circular buffer
    size_t log_index = 0;  // Circular buffer index
    size_t valid_count = 0;
    bool enabled = false;
    std::string logger_name; // Unique name for identifying the logger instance

public:
    /**
     * Register logger instances globally (Called in constructor)
     */
    Logger(const std::string& name, const bool _enabled = true) : logger_name(name), enabled(_enabled) {
    }

    void enable(const bool _enabled) {
        enabled = _enabled;
    }

    /**
     * Push a log entry into the buffer
     * @param type The log type
     * @param data The data to log (x, y, heading, time)
     * This is a fixed-size array of 4 elements
     * The data is stored in a circular buffer fashion
     * The log index wraps around when it reaches the maximum size
     */
    void push_log(LogType type, std::array<float, LOG_SIZE> data) {
        if (writingToFile || !enabled) return;
        logs[log_index] = {type, int(pros::millis()), data[0], data[1], data[2], data[3]};  // Store the log entry
        log_index = (log_index + 1) % MAX_LOG_ENTRIES;  // Circular wrap-around
        valid_count++;
        if (valid_count > MAX_LOG_ENTRIES) valid_count = MAX_LOG_ENTRIES;  // Ensure valid count does not exceed max
    }

    void dump_file(FILE* log_file) {
        char log_string[256];

        for (size_t i = 0; i < valid_count; i++) {
            const LogElement& log = logs[i];

            switch (log.type) {
                case LogType::POSITION_EXPECTED:
                    sprintf(log_string, "%s:, Expected, %d, %f, %f, %f\n", logger_name.c_str(), log.time, log.a, log.b, log.c);
                    break;
                case LogType::POSITION_REAL:
                    sprintf(log_string, "%s, Real, %d, %f, %f, %f\n", logger_name.c_str(), log.time, log.a, log.b, log.c);
                    break;
                case LogType::LOOP_TIME:
                    sprintf(log_string, "%s, LoopTime, %d, %f\n", logger_name.c_str(), log.time, log.a);
                    break;
                case LogType::DISTANCE_SENSOR:
                    sprintf(log_string, "%s, DistanceSensor, %d, %f, %f, %f, %f\n", logger_name.c_str(), log.time, log.a, log.b, log.c, log.d);
                    break;
                case LogType::DELTA_MOVEMENT:
                    sprintf(log_string, "%s, DeltaMovement, %d, %f, %f\n", logger_name.c_str(), log.time, log.a, log.b);
                    break;
                case LogType::PARTICLE_POSITION:
                    sprintf(log_string, "%s, ParticlePosition, %d, %f, %f\n", logger_name.c_str(), log.time, log.a, log.b);
                    break;
                case LogType::UNIQUE_PARTICLES:
                    sprintf(log_string, "%s, UniqueParticles, %d, %f\n", logger_name.c_str(), log.time, log.a);
                    break;
                case LogType::STANDARD_DEVIATION:
                    sprintf(log_string, "%s, StandardDeviation, %d, %f\n", logger_name.c_str(), log.time, log.a);
                    break;
                case LogType::DEVIATION_AND_UNIQUE:
                    sprintf(log_string, "%s, DeviationAndUnique, %d, %f, %f, %f, %f\n", logger_name.c_str(), log.time, log.a, log.b, log.c, log.d);
                    break;
                case LogType::DELTA_MOVEMENT_AND_LOOP_TIME:
                    sprintf(log_string, "%s, DeltaMovementAndLoopTime, %d, %f, %f, %f\n", logger_name.c_str(), log.time, log.a, log.b, log.c);
                    break;
                case LogType::OBJECT_SIZE:
                    sprintf(log_string, "%s, ObjectSize, %d, %f, %f, %f, %f\n", logger_name.c_str(), log.time, log.a, log.b, log.c, log.d);
                    break;
                default:
                    break;
            }

            fputs(log_string, log_file);
        }

        std::cout << "Dumped logs for " << logger_name << std::endl;
    }
};

extern void dump_all();

constexpr size_t distanceLogSize = 10;
constexpr size_t robotLogSize = 10;
constexpr size_t autonLogSize = 10;
constexpr size_t pfLogSize = 10;

extern Logger<distanceLogSize> distanceLogger;
extern Logger<robotLogSize> robotLogger;
extern Logger<autonLogSize> autonLogger;
extern Logger<pfLogSize> pfLogger;

