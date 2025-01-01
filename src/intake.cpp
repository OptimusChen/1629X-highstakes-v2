#include "intake.hpp"
#include "lib/controller/pid.hpp"
#include "controls.hpp"
#include <iostream>

pros::Mutex movingMutex;
pros::Mutex voltsMutex;

void Intake::initialize() {
    if (initialized) return;
    optical = new Optical(OPTICAL);
    optical->set_led_pwm(100);

    hooks = new Motor(HOOKS);
    hooks->set_reversed(true);
    hooks->set_gearing(E_MOTOR_GEAR_BLUE);

    antijam = true;
    initialized = true;
}

void Intake::update() {
    auto opticalMeasure = optical->get_hue();

    if (color == BLUE && ((opticalMeasure > 0) && (opticalMeasure < 30)) && color_sort) {
        color_sort = false;
        Task t{[&] {
            delay(500);
            hooks->move(-127);
            delay(100);
            hooks->move(volts);
            color_sort = true;
        }};   
    }

    // 600, 175
    // 300, 400

    if (color == RED && ((opticalMeasure > 200) && (opticalMeasure < 230)) && color_sort) {
        color_sort = false;
        Task t{[&] {
            float toDelay = 150;
            delay(toDelay);
            hooks->move(0);
            hooks->move(-127);
            delay(300);
            hooks->move(volts);
            this->color_sort = true;
        }};   
    }

    if (antijam && (arm->armTarget != LOAD) && (arm->armTarget != MID)) {
        movingMutex.take();
        bool b = moving;
        movingMutex.give();
        if (b && (abs(hooks->get_actual_velocity()) < 10)) {
            counter++;

            if (counter >= 2) {
                antijam = false;
                counter = 0;
                Task t{[&] {
                    hooks->move(-127);
                    delay(150);
                    hooks->move(volts);
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
    hooks->move(power);
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
    hooks->move(-power);
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
    hooks->move(0);
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