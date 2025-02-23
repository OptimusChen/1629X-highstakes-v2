/**
 * Implementation of the logging library
 */
#include "logging.hpp"
#include <vector>
#include "pros/rtos.hpp"
#include <iostream>
#include <string>

/** The logging buffer. */
static std::vector<LogElement> logs;

namespace logging {
    /**
     * Push a log to the buffer
     * @param type The type of the log
     * @param x The x position
     * @param y The y position
     * @param heading The heading
     * @param time The time
     */
    void push_log(const LogType type, const float x, const float y, const float heading, const float time) {
        logs.push_back({type, x, y, heading, static_cast<float>(pros::millis())});
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
     * Dump the logs to a file
     */
    void dump() {
        /* idx of the file and increment by 1. */
        uint32_t idx = get_log_file_idx();
        idx ++;
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
        for (auto log : logs) {
            switch (log.type)
            {
            case LogType::POSITION_EXPECTED:
                sprintf(log_string, "Expected, %f, %f, %f\n", log.x, log.y, log.heading);
                break;
            case LogType::POSITION_REAL:
                sprintf(log_string, "Real, %f, %f, %f\n", log.x, log.y, log.heading);
                break;
            default:
                break;
            }

            fputs(log_string, log_file);
        }

        fclose(log_file);
    }
}
