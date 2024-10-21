#include "controls.hpp"
#include "main.h"


namespace controls {
   auto corner_arm = ADIDigitalOut(CORNER_ARM);
   auto intake_lift = ADIDigitalOut(INTAKE_LIFT);
   auto mogo = ADIDigitalOut(MOGO);

   void outtake() {
       c::motor_move(HOOKS, -127);
   }

   void intake(int speed) {
       c::motor_move(HOOKS, speed);
   }

   void begin_intake(int duration, bool async, std::function<void()> callback, int speed) {
       if (async) {
           pros::Task intake_task([=]() {
               begin_intake(duration, false, nullptr, speed);


               if (callback) callback();
           });
           return;
       }


       intake();


       if (duration == -1) return;


       delay(duration);


       stop_intake();
   }


   void stop_intake() {
       c::motor_move_velocity(HOOKS, 0);
   }


   void clamp_mogo() {
       mogo.set_value(true);
   }


   void release_mogo() {
       mogo.set_value(false);
   }


   void raise_intake() {
       intake_lift.set_value(true);
   }


   void lower_intake() {
       intake_lift.set_value(false);
   }


   void activate_corner_arm() {
       corner_arm.set_value(true);
   }


   void lower_corner_arm() {
       corner_arm.set_value(false);
   }
}
