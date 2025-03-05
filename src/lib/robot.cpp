#include "lib/robot.hpp"
#include "lib/util.hpp"
#include "lib/controller/pid.hpp"
#include "lib/controller/feedForward.hpp"
#include "lib/bezier.h"
#include "lib/motionProfiling.hpp"
#include "lib/controller/ramsete.hpp"
#include "controls.hpp"
#include <math.h>
#include <iomanip>
#include "lib/logging.hpp"

using namespace lib;

ADIDigitalOut mogo(MOGO);
ADIDigitalOut rush_left(DOINKER_LEFT);
ADIDigitalOut rush_right(DOINKER_RIGHT);
ADIDigitalOut lift_intake(INTAKE_LIFT);
ADIDigitalOut csortpiston(SORTING_PISTON);

Motor hooks(HOOKS);

Robot::Robot(Odom* odom, MotorGroup* left, MotorGroup* right, PID* lateral, PID* angular, PID* angular_slow) {
    this->odometry = odom;

    this->left = left;
    this->right = right;

    this->lateral = lateral;
    this->angular = angular;
    this->angular_slow = angular_slow;
}

void Robot::set_pose(float x, float y, float theta, bool radians) {
    odometry->set_position(x, y, theta, radians);
}

void Robot::set_pose_mode(int mode) {
    this->poseMode = mode;

    odometry->mcl = mode == MCL;
}

Pose Robot::get_pose() {
    if (this->poseMode == MCL) {
        auto pred = particleFilter->getPrediction();
        float cartesianX = -pred.y() * metre.Convert(inch);
        float cartesianY = pred.x() * metre.Convert(inch);
        return Pose(cartesianX, cartesianY, odometry->get_pose().theta, true);
    }
    return odometry->get_pose();
}

void Robot::calibrate() {
    if (!poseSet) return;
    odometry->inertial->reset(true);

    odometry->start();
}

void Robot::set_constants(float wheelDiameter, int rpm, float mass, float trackWidth, float friction_coef) {
    this->wheelDiameter = wheelDiameter;
    this->rpm = rpm;
    this->mass = mass;
    this->trackWidth = trackWidth;
    this->friction_coef = friction_coef;
}

void Robot::add_subsystem(Subsystem* subsystem) {
    subsystems.push_back(subsystem);
}

Subsystem* Robot::get_subsystem(const std::string& name) {
    for (auto* subsystem : subsystems) {
        if (typeid(*subsystem).name() == name) {
            return subsystem;
        }
    }
    return nullptr;
}

void Robot::set_pf(loco::ParticleFilter<PARTICLES>* particleFilter) {
    this->particleFilter = particleFilter;
}

void Robot::set_mogo(bool value) {
    mogo.set_value(value);
}

void Robot::set_rush_arm_left(bool value) {
    rush_left.set_value(value);
}

void Robot::set_rush_arm_right(bool value) {
    rush_right.set_value(value);
}

void Robot::set_lift_intake(bool value) {
    lift_intake.set_value(value);
}

constexpr float DRIVE_RATIO = 48.0/36.0; // EX: 36 tooth driving gear to 48 tooth driven gear.
constexpr QLength WHEEL_RADIUS = 2.75_in/2.0; // Wheel radius
constexpr float DRIVE_NOISE = 0.35; // The desired amount in % of noise on the drive
constexpr Angle ANGLE_NOISE = 8_deg; // The noise on the angle that's desired

std::ranlux24_base de;

QLength lastLeft, lastRight;

QLength getDistance(const pros::MotorGroup* motor) {
    QLength totalPosition = 0.0;

    for (double position : motor->get_position_all()) {
        totalPosition += position / DRIVE_RATIO * 2.0 * M_PI * WHEEL_RADIUS;
    }

    return totalPosition/motor->size();
}

void Robot::reset_particle_filter(float x, float y) {
    Eigen::Vector2f mean(y * inch.Convert(metre), -x * inch.Convert(metre));

    Eigen::Matrix2f covariance;
    covariance << 0.2f, 0.0f,
                0.0f, 0.2f;

    particleFilter->initNormal(mean, covariance, false);

    for (auto particle : particleFilter->getParticles()) {
        robotLogger.push_log(LogType::PARTICLE_POSITION, {particle.x(), particle.y(), particle.z(), -1});
    }
}

void Robot::initialize_particle_filter() {
    reset_particle_filter(get_pose().x, get_pose().y);

    pros::Task locoTask = pros::Task([&]() {
        uint32_t start_time = 0;

        // Run localization forever
        while (true) {
            // Store the start time to ensure that the time between updates remains consistent
            start_time = pros::millis();

            // Store the current distance of the drivetrain
            const QLength leftLength = getDistance(left);
            const QLength rightLength = getDistance(right);

            // Calculate the change from the previous position
            const QLength leftChange = leftLength - lastLeft;
            const QLength rightChange = rightLength - lastRight;
            
            // Store the current value as the last value for next frame
            lastLeft = leftLength;
            lastRight = rightLength;

            // Calculate the average movement, this is a cheaper way to get the movement of the drive at the center for
            // skid-steer based mechanics
            auto avg = (leftChange + rightChange) / 2.0;

            // Define the distributions to add noise to the sensor readings
            std::uniform_real_distribution avgDistribution(avg.getValue() - DRIVE_NOISE * avg.getValue(),
                                                        avg.getValue() + DRIVE_NOISE * avg.getValue());
            std::uniform_real_distribution angleDistribution(
                particleFilter->getAngle().getValue() - ANGLE_NOISE.getValue(),
                particleFilter->getAngle().getValue() + ANGLE_NOISE.getValue());

            // Calculate noisy sensor readings
            const auto noisy = avgDistribution(de);
            const auto angle = angleDistribution(de);

            auto change = Eigen::Rotation2Df(angle) * Eigen::Vector2f({noisy, 0.0});
            robotLogger.push_log(LogType::DELTA_MOVEMENT, {change.x(), change.y(), -1, -1});

            particleFilter->update([&]() mutable {
                // Calculate the translation with the sensor readings
                return change;
            }, pros::millis() * millisecond);
      
            robotLogger.push_log(LogType::LOOP_TIME, {float(pros::millis() - start_time), -1, -1, -1});
            robotLogger.push_log(LogType::POSITION_REAL, {particleFilter->getPrediction().x(), particleFilter->getPrediction().y(), particleFilter->getPrediction().z(), -1});

            pros::c::task_delay_until(&start_time, 10);
        }
    });
}

void Robot::reset_position(SensorOrientation fwdbck, SensorOrientation leftright, int quadrant) {
    // auto sensors = particleFilter->getSensors();
    
    // // Retrieve the selected sensors
    // float primary_sensor = sensors[fwdbck.sensorIndex]->get_measurement();
    // float secondary_sensor = sensors[leftright.sensorIndex]->get_measurement();

    // float estimated_x = 0.0, estimated_y = 0.0;

    // constexpr int wall = 1.78308 * metre.Convert(inch); // The length of the wall in meters

    // // Determine estimated position based on quadrant
    // switch (quadrant) {
    //     case 1: // (+x, +y)
    //         //whatevs
    //     case 2: // (-x, +y)
    //     //whatevs
    //         break;
    //     case 3: // (-x, -y)
    //         float newX = 
    //         break;
    //     case 4: // (+x, -y)
    //     //whatevs
    //         break;
    //     default:
    //         return; // Invalid quadrant
    // }

    // // Convert inches to meters
    // float x_meters = estimated_x * inch.Convert(metre);
    // float y_meters = estimated_y * inch.Convert(metre);

    // // Reset the particle filter with the new estimated position
    // reset_particle_filter(x_meters, y_meters);

}

void Robot::set_brake_mode(motor_brake_mode_e_t mode) {
    left->set_brake_mode(mode);
    right->set_brake_mode(mode);
}

void Robot::set_use_slow_angular(bool value) {
    useSlowAngular = value;
}

void Robot::set_color_sort_piston(bool value) {
    csortpiston.set_value(value);
}