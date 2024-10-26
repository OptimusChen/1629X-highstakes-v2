#pragma once

#include "pros/misc.h"
#include "pros/motors.h"


#include "pros/motor_group.hpp"


#define L_DRIVE_FRONT 16
#define L_DRIVE_MID 17
#define L_DRIVE_BACK 18


#define R_DRIVE_FRONT 13
#define R_DRIVE_MID 14
#define R_DRIVE_BACK 15


#define HOOKS 6


#define MOGO 1
#define CORNER_ARM 2
#define INTAKE_LIFT 3


#define ARM 5


#define VERTICAL 1
#define HORIZONTAL 10


#define INERTIAL_PORT 11


#define ARM_ROT 9


using namespace pros;


namespace controls {
   void clamp_mogo();
   void release_mogo();


   void raise_intake();
   void lower_intake();
  
   void intake(int speed = 127);
   void outtake();
   void stop_intake();


   void activate_corner_arm();
   void lower_corner_arm();


   void begin_intake(int duration, bool async, std::function<void()> callback, int speed = 127);
}
