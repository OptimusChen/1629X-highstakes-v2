#include "intake.hpp"
#include "lib/controller/pid.hpp"
#include "controls.hpp"
#include <iostream>

ADIDigitalOut sortingPiston(SORTING_PISTON);

void Intake::initialize() {
    if (initialized) return;
    optical = new Optical(OPTICAL);
    colorSortOptical = new Optical(COLOR_SORT_OPTICAL);
    
    optical->set_integration_time(10);
    optical->set_led_pwm(100);
    colorSortOptical->set_integration_time(10);
    colorSortOptical->set_led_pwm(100);

    hooks.set_gearing(E_MOTOR_GEAR_BLUE);
    hooks.set_reversed(true);
    hooks.tare_position();

    hooks.set_encoder_units(E_MOTOR_ENCODER_DEGREES);

    antijam = true;
    initialized = true;
}

void Intake::update() {
    auto opticalMeasure = colorSortOptical->get_hue();
    int off = 100;
    bool loading = arm->armTarget == LOAD;
    if (loading) {
        off = 100;
    }
    
    if (color == BLUE && ((opticalMeasure > 0) && (opticalMeasure < 30)) && color_sort && !toSort) {
        toSort = true;
        hooks.tare_position();
        wrongDetected = hooks.get_position();
        color_sort = false;
        std::cout << "sort1" << std::endl;
    }

    if (color == RED && ((opticalMeasure > 180) && (opticalMeasure < 215)) && color_sort && !toSort && colorSortOptical->get_proximity() > 120) {
        this->toSort = true;

        pros::Task ejectRingTask([&] {
          hooks.move(127);
          pros::delay(50);
          hooks.move(-127); // we suddenly stop the intake right before the ring reaches the top, so
                                              // it's inertia flings it off the hook before it can score onto the mogo

          pros::delay(200);
          hooks.move(127);
          this->toSort = false;
        });

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
                    hooks.move(-40);
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
    if (toSort) {
        return;
    }
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
    return optical->get_proximity() > threshold;
}

void Intake::set_stop_condition(std::function<bool()> condition) {
    stop_condition = condition;
}

void Intake::set_anti_jam(bool antijam) {
    this->antijam = antijam;
}