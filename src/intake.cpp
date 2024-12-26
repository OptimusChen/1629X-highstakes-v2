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
    
    // std::cout << opticalMeasure.red << ", " << opticalMeasure.green << ", " << opticalMeasure.blue << std::endl;
    // std::cout << optical->get_proximity() << std::endl;

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
}

void Intake::set_color(int color) {
    this->color = color;
}

void Intake::forwards() {
    hooks->move(reversed ? -127 : 127);
}

void Intake::backwards() {
    hooks->move(-127);
}

void Intake::stop() {
    hooks->move(0);
}