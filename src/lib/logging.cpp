#include "logging.hpp"
#include <iostream>
#include <fstream>
#include <cstdio>
#include <vector>

#include "pros/rtos.hpp"

Logger<distanceLogSize> distanceLogger("distance_sensors");
Logger<robotLogSize> robotLogger("robot");
Logger<autonLogSize> autonLogger("auton");
Logger<pfLogSize> pfLogger("pf");
bool writingToFile = false;

/**
 * Get the index of the log file
 * @return The index of the log file
 */
static uint8_t get_log_file_idx() {
    char read_buf[100]=""; // This just needs to be larger than the contents of the file
    FILE* index_file = fopen("/usd/index.txt", "r");
    if (index_file == NULL) {
        printf("Failed to open index file\n");
        return 0;
    }
    fread(read_buf, 1, 50, index_file);
    fclose(index_file);
    if (read_buf[0] == '\0') {
        return 0;
    }
    return std::stoi(read_buf);
}

/**
 * Write the index of the log file
 * @param idx The index to write
 */
static void write_log_file_idx(uint8_t idx) {
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
void dump_all() {
    writingToFile = true;
    
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

    distanceLogger.dump_file(log_file);
    robotLogger.dump_file(log_file);
    autonLogger.dump_file(log_file);
    pfLogger.dump_file(log_file);

    fclose(log_file);
    std::cout << "Dumped logs to file " << filename << std::endl;
}