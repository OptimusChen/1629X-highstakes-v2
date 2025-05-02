#pragma once

#include "pros/misc.h"
#include "pros/motors.h"

#include "pros/motor_group.hpp"

#include "localization/particleFilter.h"
#include "localization/distance.h"

#define L_DRIVE_FRONT 6
#define L_DRIVE_MID 8
#define L_DRIVE_BACK 7

#define R_DRIVE_FRONT 4
#define R_DRIVE_MID 3
#define R_DRIVE_BACK 5

#define HOOKS 20

#define R_DISTANCE 11
#define L_DISTANCE 19
#define B_DISTANCE 14
#define F_DISTANCE 15

#define OPTICAL 0
#define COLOR_SORT_OPTICAL 13

#define ARM_PISTON_LEFT 5
#define ARM_PISTON_RIGHT 1

#define SORTING_PISTON 5
#define INTAKE_LIFT 6

#define DOINKER_LEFT 7
#define DOINKER_RIGHT 8
#define MOGO 5

#define CORNER_ARM 0

#define VERTICAL 14
#define HORIZONTAL 21

#define INERTIAL_PORT 2

#define LB_ROTATION 9
#define ARM_LEFT 19
#define ARM_RIGHT 18

#define LADYBROWN 10
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
