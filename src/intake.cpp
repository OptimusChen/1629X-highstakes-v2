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
}

void Intake::update() {
    auto opticalMeasure = optical->get_rgb();
    
    if (color == BLUE && opticalMeasure.red > 150.0f && !reversed && color_sort) {
        color_sort = false;
        Task {[&] {
            delay(100);
            reversed = true;
            delay(100);
            reversed = false;
            color_sort = true;
        }};   
    }

    if (color == RED && opticalMeasure.blue > 100.0f && !reversed && color_sort) {
        color_sort = false;
        Task {[&] {
            delay(100);
            reversed = true;
            delay(100);
            reversed = false;
            color_sort = true;
        }};   
    }

    if (moving && antijam && (abs(hooks->get_actual_velocity()) < 10)) {
        counter++;

        if (counter > 5) {
            antijam = false;
            Task {[&] {
                reversed = true;
                delay(250);
                reversed = false;
                antijam = true;
            }};
        }
    } else {
        counter = 0;
    }

    if (stop_condition && stop_condition()) {
        stop();
    }
}

void Intake::set_color(int color) {
    this->color = color;
}

void Intake::forwards(int power) {
    hooks->move(reversed ? -power : power);
    moving = true;
}

void Intake::backwards(int power) {
    hooks->move(-power);
    moving = true;
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