#pragma once

#include "pros/misc.h"
#include "pros/motors.h"

#include "pros/motor_group.hpp"

#define L_DRIVE_FRONT 12
#define L_DRIVE_MID 15
#define L_DRIVE_BACK 3

#define R_DRIVE_FRONT 13
#define R_DRIVE_MID 14
#define R_DRIVE_BACK 2

#define HOOKS 1

#define R_DISTANCE 4
#define L_DISTANCE 8
#define B_DISTANCE 9

#define ARM_PISTON_LEFT 5
#define ARM_PISTON_RIGHT 8
#define INTAKE_LIFT 1
#define DOINKER 0
#define MOGO_LEFT 2
#define MOGO_RIGHT 4

#define MOGO 0
#define CORNER_ARM 0


#define ARM 5


#define VERTICAL 10
#define HORIZONTAL 21


#define INERTIAL_PORT 6


#define LEFT_ROTATION 20
#define RIGHT_ROTATION 19

#define LEFT_ARM 6
#define RIGHT_ARM 7

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
