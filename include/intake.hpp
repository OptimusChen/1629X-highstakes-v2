#pragma once

#include "lib/subsystem.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"

using namespace pros;
using namespace lib;

#define BLUE 0
#define RED 1
#define NONE -1

class Intake : public Subsystem {
public:
    bool reversed = false;
    bool color_sort = false;
    int color = BLUE;
    Optical* optical;
    Motor* hooks;

    std::function<bool()> stop_condition;

    void initialize() override;
    void update() override;
    void stop() override;

    void set_color(int color);
    void forwards(int power = 127);
    void backwards(int power = 127);
    void set_stop_condition(std::function<bool()> condition);
    bool detected_ring();
};