#pragma once

#include "pros/misc.h"
#include "pros/motors.h"

#include "pros/motor_group.hpp"

#include "localization/particleFilter.h"
#include "localization/distance.h"

#define L_DRIVE_FRONT 8
#define L_DRIVE_MID 7
#define L_DRIVE_BACK 10

#define R_DRIVE_FRONT 2
#define R_DRIVE_MID 4
#define R_DRIVE_BACK 1

#define HOOKS 5

#define R_DISTANCE 16
#define L_DISTANCE 17
#define B_DISTANCE 14
#define F_DISTANCE 15

#define OPTICAL 3

#define ARM_PISTON_LEFT 5
#define ARM_PISTON_RIGHT 1

#define INTAKE_LIFT 4

#define DOINKER_LEFT 2
#define DOINKER_RIGHT 3
#define MOGO 1

#define CORNER_ARM 0

#define VERTICAL 10
#define HORIZONTAL 21

#define INERTIAL_PORT 6

#define LB_ROTATION 13
#define ARM_LEFT 19
#define ARM_RIGHT 18

#define LADYBROWN 19
#define L_SWITCH 1

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
