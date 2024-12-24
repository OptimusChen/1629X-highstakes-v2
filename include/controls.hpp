#pragma once

#include "pros/misc.h"
#include "pros/motors.h"

#include "pros/motor_group.hpp"

#include "localization/particleFilter.h"
#include "localization/distance.h"

#define L_DRIVE_FRONT 12
#define L_DRIVE_MID 11
#define L_DRIVE_BACK 20

#define R_DRIVE_FRONT 2
#define R_DRIVE_MID 1
#define R_DRIVE_BACK 10

#define HOOKS 15

#define R_DISTANCE 8
#define L_DISTANCE 18
#define B_DISTANCE 4
#define F_DISTANCE 3

#define OPTICAL 21

#define ARM_PISTON_LEFT 5
#define ARM_PISTON_RIGHT 1

#define INTAKE_LIFT 3

#define DOINKER 0

#define MOGO 8

#define CORNER_ARM 0

#define VERTICAL 10
#define HORIZONTAL 21

#define INERTIAL_PORT 13

#define LB_ROTATION 5
#define ARM_LEFT 14
#define ARM_RIGHT 6

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
