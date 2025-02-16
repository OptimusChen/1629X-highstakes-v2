#include "intake.hpp"
#include "lib/controller/pid.hpp"
#include "controls.hpp"
#include <iostream>

void Intake::initialize() {
    if (initialized) return;
    optical.set_integration_time(5);
    optical.set_led_pwm(100);

    hooks.set_gearing(E_MOTOR_GEAR_BLUE);
    hooks.set_reversed(true);
    hooks.tare_position();

    antijam = true;
    initialized = true;
}

void Intake::update() {
    auto opticalMeasure = optical.get_hue();
    int off = 550;
    bool loading = arm->armTarget == LOAD;
    if (loading) {
        off = 400;
    }

    // if (color == BLUE && ((opticalMeasure > 0) && (opticalMeasure < 30)) && color_sort) {
    //     color_sort = false;
    //     Task t{[&] {
    //         delay(500);
    //         hooks.move(-127);
    //         delay(100);
    //         voltsMutex.take();
    //         const auto _volts = volts;
    //         voltsMutex.give();
    //         hooks.move(_volts);
    //         color_sort = true;
    //     }};   
    // }

    // 600, 175
    // 300, 400
    
    if (color == BLUE && ((opticalMeasure > 0) && (opticalMeasure < 30)) && color_sort && !toSort) {
        toSort = true;
        hooks.tare_position();
        wrongDetected = hooks.get_position();
        color_sort = false;
        std::cout << "sort1" << std::endl;
    }

    if (color == RED && ((opticalMeasure > 170) && (opticalMeasure < 220)) && color_sort && !toSort) {
        toSort = true;
        hooks.tare_position();
        wrongDetected = hooks.get_position();
        color_sort = false;
        std::cout << "sort1" << std::endl;
    }

    // std::cout << opticalMeasure << std::endl;;
    
    if (toSort && !color_sort && abs(hooks.get_position() - (wrongDetected + off)) < 20) {
        std::cout << "sort2" << std::endl;
        color_sort = true;

        Task t([&] {
            hooks.move(-127);
            reversed = true;
            delay(loading ? 600 : 300);
            voltsMutex.take();
            const auto _volts = volts;
            voltsMutex.give();
            hooks.move(_volts);
            hooks.tare_position();

            reversed = false;
            toSort = false;
        });  
    }

    if (toSort && hooks.get_position() > 2*off) {
        color_sort = true;
        toSort = false;
    }

    if (!reversed && antijam && (arm->armTarget != LOAD) && (arm->armTarget != MID)) {
        movingMutex.take();
        bool b = moving;
        movingMutex.give();
        if (b && (abs(hooks.get_actual_velocity()) < 10)) {
            counter++;

            if (counter >= 4) {
                antijam = false;
                counter = 0;
                Task t{[&] {
                    hooks.move(-127);
                    delay(150);
                    voltsMutex.take();
                    const auto _volts = volts;
                    voltsMutex.give();
                    hooks.move(_volts);
                    antijam = true;
                }};
            }
        } else {
            counter = 0;
        }
    }

    if (stop_condition && stop_condition()) {
        stop();
    }
}

void Intake::set_color(int color) {
    this->color = color;
}

void Intake::forwards(int power) {
    if (reversed) power = -power;
    hooks.move(power);
    voltsMutex.take();
    volts = power;
    voltsMutex.give();
    if (!antijam) {
        movingMutex.take();
        moving = true;
        movingMutex.give();
        return;
    }
    Task t{[&] {
        delay(100);
        movingMutex.take();
        moving = true;
        movingMutex.give();
    }};
}

void Intake::backwards(int power) {
    hooks.move(-power);
    voltsMutex.take();
    volts = -power;
    voltsMutex.give();
    if (!antijam) {
        movingMutex.take();
        moving = true;
        movingMutex.give();
        return;
    }
    Task t{[&] {
        delay(100);
        movingMutex.take();
        moving = true;
        movingMutex.give();
    }};
}

void Intake::stop() {
    hooks.move(0);
    moving = false;
}

bool Intake::detected_ring(int threshold) {
    return optical.get_proximity() > threshold;
}

void Intake::set_stop_condition(std::function<bool()> condition) {
    stop_condition = condition;
}

void Intake::set_anti_jam(bool antijam) {
    this->antijam = antijam;
}