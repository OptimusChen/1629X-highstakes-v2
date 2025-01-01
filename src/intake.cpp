#include "intake.hpp"
#include "lib/controller/pid.hpp"
#include "controls.hpp"
#include <iostream>

void Intake::initialize() {
    optical = new Optical(OPTICAL);
    optical->set_led_pwm(100);

    hooks = new Motor(HOOKS);
    hooks->set_reversed(true);
    hooks->set_gearing(E_MOTOR_GEAR_BLUE);

    antijam = true;
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

    

    if (color == RED && ((opticalMeasure > 200) && (opticalMeasure < 250)) && color_sort) {
        color_sort = false;
        Task t{[&] {
            float toDelay = (-3/4)*hooks->get_actual_velocity() + 625;
            delay(toDelay);
            hooks->move(0);
            hooks->move(-127);
            delay(300);
            hooks->move(volts);
            this->color_sort = true;
        }};   
    }

    if (antijam && (arm->armTarget != LOAD)) {
        if (moving && (abs(hooks->get_actual_velocity()) < 10)) {
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
    volts = power;
    if (!antijam) {
        moving = true;
        return;
    }
    Task t{[&] {
        delay(100);
        moving = true;
    }};
}

void Intake::backwards(int power) {
    hooks->move(-power);
    volts = -power;
    if (!antijam) {
        moving = true;
        return;
    }
    Task t{[&] {
        delay(100);
        moving = true;
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