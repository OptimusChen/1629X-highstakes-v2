#pragma once

#include "lib/subsystem.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/optical.h"
#include "arm.hpp"
#include "controls.hpp"

using namespace pros;
using namespace lib;

#define BLUE 0
#define RED 1
#define NONE -1

#define STAGE_1 40
#define STAGE_2 240

class Intake : public Subsystem {
public:
    bool reversed = false;
    bool color_sort = true;
    bool antijam = false;
    bool moving = false;
    int color = RED;
    bool sortNextRing = false;
    Optical* optical;
    Optical* colorSortOptical;
    Motor hooks{HOOKS};
    Arm* arm;

    bool toSort = false;
    double wrongDetected = 0;

    std::function<bool()> stop_condition;

    void initialize() override;
    void update() override;
    void stop() override;

    void set_color(int color);
    void set_anti_jam(bool antijam);
    void forwards(int power = 127);
    void backwards(int power = 127);
    void set_stop_condition(std::function<bool()> condition);
    bool detected_ring(int threshold = STAGE_2);
private:
    int counter = 0;
    int volts = 0;
    pros::Mutex movingMutex {};
    pros::Mutex voltsMutex {}; 
};