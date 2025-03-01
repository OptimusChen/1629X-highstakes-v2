#include "logging.hpp"
#include <iostream>
#include <fstream>
#include <cstdio>
#include <vector>

#include "pros/rtos.hpp"

// Collection of all logger instances
static std::array<Logger&, MAX_LOGGERS> all_loggers;

/**
 * Register logger instances globally (Called in constructor)
 */
Logger::Logger(const std::string& name) : logger_name(name) {
    all_loggers.at(logger_count) = *this;  // Register this instance
    logger_count++;
    if (logger_count >= MAX_LOGGERS) {
        std::cout << "Maximum number of loggers reached. Cannot register more." << std::endl;
    }
}

/**
 * Push a log entry into the buffer
 * @param type The log type
 * @param data The data to log (x, y, heading, time)
 * This is a fixed-size array of 4 elements
  * The data is stored in a circular buffer fashion
  * The log index wraps around when it reaches the maximum size
 */
void Logger::push_log(LogType type, std::array<float, LOG_SIZE> data) {
    logs[log_index] = {type, pros::millis(), data[0], data[1], data[2], data[3]};  // Store the log entry
    log_index = (log_index + 1) % MAX_LOG_ENTRIES;  // Circular wrap-around
    valid_count++;
    if (valid_count > MAX_LOG_ENTRIES) valid_count = MAX_LOG_ENTRIES;  // Ensure valid count does not exceed max
}

/**
 * Get the index of the log file
 * @return The index of the log file
 */
static uint32_t get_log_file_idx() {
    char read_buf[100]; // This just needs to be larger than the contents of the file
    FILE* index_file = fopen("/usd/index.txt", "r");
    if (index_file == NULL) {
        printf("Failed to open index file\n");
        return 0;
    }
    fread(read_buf, 1, 50, index_file);
    fclose(index_file);
    return std::stoi(read_buf);
}

/**
 * Write the index of the log file
 * @param idx The index to write
 */
static void write_log_file_idx(uint32_t idx) {
    FILE* index_file = fopen("/usd/index.txt", "w");
    if (index_file == NULL) {
        printf("Failed to open index file\n");
        return;
    }
    fputs(std::to_string(idx).c_str(), index_file);
    fclose(index_file);
}

/**
 * Dump all logs from all instances to a single file
 */
void Logger::dump_all() {
    uint8_t idx = get_log_file_idx();
    idx++;
    write_log_file_idx(idx);

    /* generate the new file name based on index */
    char filename[100];
    sprintf(filename, "/usd/%d.txt", idx);

    /* Dump the log to the log file. */
    FILE* log_file = fopen(filename, "w");
    if (log_file == NULL) {
        printf("Failed to open file for writing\n");
        return;
    } 

    char log_string[256];

    // Iterate over all registered loggers
    for (Logger& logger : all_loggers) {
        for (size_t i = 0; i < logger.valid_count; i++) {
            const LogElement& log = logger.logs[i];

            switch (log.type) {
                case LogType::POSITION_EXPECTED:
                    sprintf(log_string, "%s:, Expected, %f, %f, %f, %f\n", logger.logger_name.c_str(), log.time, log.a, log.b, log.c);
                    break;
                case LogType::POSITION_REAL:
                    sprintf(log_string, "%s, Real, %f, %f, %f, %f\n", logger.logger_name.c_str(), log.time, log.a, log.b, log.c);
                    break;
                case LogType::LOOP_TIME:
                    sprintf(log_string, "%s, LoopTime, %f, %f\n", logger.logger_name.c_str(), log.time, log.a);
                    break;
                case LogType::DISTANCE_SENSOR:
                    sprintf(log_string, "%s, DistanceSensor, %f, %f, %f, %f, %f\n", logger.logger_name.c_str(), log.time, log.a, log.b, log.c, log.d);
                    break;
                case LogType::DELTA_MOVEMENT:
                    sprintf(log_string, "%s, DeltaMovement, %f, %f, %f\n", logger.logger_name.c_str(), log.time, log.a, log.b);
                    break;
                default:
                    break;
            }

            fputs(log_string, log_file);
        }
    }

    fclose(log_file);
}
