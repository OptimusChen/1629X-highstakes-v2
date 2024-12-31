#pragma once

#include "pros/misc.h"
#include "pros/motors.h"

#include "pros/motor_group.hpp"

#include "localization/particleFilter.h"
#include "localization/distance.h"

#define L_DRIVE_FRONT 3
#define L_DRIVE_MID 2
#define L_DRIVE_BACK 5

#define R_DRIVE_FRONT 13
#define R_DRIVE_MID 14
#define R_DRIVE_BACK 11

#define HOOKS 1

#define R_DISTANCE 9
#define L_DISTANCE 8
#define B_DISTANCE 12
#define F_DISTANCE 16

#define OPTICAL 6

#define ARM_PISTON_LEFT 5
#define ARM_PISTON_RIGHT 1

#define INTAKE_LIFT 3

#define DOINKER 2
#define MOGO 1

#define CORNER_ARM 0

#define VERTICAL 10
#define HORIZONTAL 21

#define INERTIAL_PORT 4

#define LB_ROTATION 17
#define ARM_LEFT 19
#define ARM_RIGHT 18

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
