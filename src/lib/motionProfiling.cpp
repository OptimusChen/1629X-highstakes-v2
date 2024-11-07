#include "lib/motionProfiling.hpp"
#include "lib/bezier.h"
#include <iostream>

using namespace lib;

ProfilePoint::ProfilePoint(double x, double y, double theta, double curvature, double t, double vel, double accel)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
    this->curvature = curvature;
    this->t = t;
    this->vel = vel;
    this->accel = accel;
}
ProfilePoint::ProfilePoint(double dist, double vel)
{
    this->dist = dist;
    this->vel = vel;
}

std::ostream& operator<<(std::ostream& os, const ProfilePoint& point) {
    os << "ProfilePoint("
       << "x: " << point.x << ", "
       << "y: " << point.y << ", "
       << "theta: " << point.theta << ", "
       << "curvature: " << point.curvature << ", "
       << "t: " << point.t << ", "
       << "vel: " << point.vel << ", "
       << "accel: " << point.accel << ", "
       << "dist: " << point.dist
       << ")";
    return os;
}

Constraints::Constraints(double max_vel, double max_acc, double friction_coef, double max_dec, double max_jerk, double track_width)
{
    this->max_vel = max_vel;
    this->max_acc = max_acc;
    this->friction_coef = friction_coef;
    this->max_dec = max_dec;
    this->max_jerk = max_jerk;
    this->track_width = track_width;
}
double Constraints::maxSpeed(double curvature)
{
    if (curvature == 0) {
        // For straight-line motion, the robot should ideally reach max velocity.
        return this->max_vel;
    }

    // Max speed limited by turning capabilities based on track width and curvature.
    double max_turn_speed = ((2.0 * this->max_vel / this->track_width) * this->max_vel) /
                            (std::fabs(curvature) * this->max_vel + (2.0 * this->max_vel / this->track_width));
    
    // Max speed limited by friction to prevent slipping.
    double max_slip_speed = std::sqrt(this->friction_coef * (1.0 / std::fabs(curvature)) * 9.81);

    return std::min(max_slip_speed, max_turn_speed);
}

std::pair<double, double> Constraints::wheelSpeeds(double angularVel, double vel)
{
    double v_left = vel - angularVel * this->track_width / 2;
    double v_right = vel + angularVel * this->track_width / 2;
    return std::make_pair(v_left, v_right);
}

ProfileGenerator::ProfileGenerator(Constraints *constraints, double dd)
{
    this->constraints = constraints;
    this->dd = dd;
}

void ProfileGenerator::generateProfile(bezier::Bezier<3> path)
{
    this->profile.clear();

    // dont ask    
    double dt = 0.01;
    double vel = 0;

    std::vector<ProfilePoint> forwardPass;
    std::vector<ProfilePoint> backwardPass;

    double t = 0;
    double curvature;
    double angular_vel = 0;
    double angular_accel = 0;
    double last_angular_vel = 0;
    double max_accel = 0;
    double theta = 0;
    bezier::Point deriv;
    bezier::Point derivSecond;

    while (t <= 1)
    {
        auto p1 = path.valueAt(t);

        deriv = path.derivative().valueAt(t);
        derivSecond = path.derivative().derivative().valueAt(t);

        theta = std::atan2(deriv.y, deriv.x);

        curvature = path.curvatureAt(deriv, derivSecond);

        double maxSpeed = constraints->maxSpeed(curvature);

        angular_vel = vel * curvature;
        angular_accel = (angular_vel - last_angular_vel) / dt;
        last_angular_vel = angular_vel;
        max_accel = this->constraints->max_acc - fabs(angular_accel * this->constraints->track_width / 2);

        double maxAchievable = vel + dt * max_accel;

        vel = std::min(maxSpeed, maxAchievable);

        double angular = vel*curvature;

        std::cout << vel << " * " << curvature << " = " << angular << std::endl;

        forwardPass.push_back(ProfilePoint(p1.x, p1.y, theta, curvature, t, vel, angular));

        double distance = vel * dt + 0.5 * max_accel * dt * dt;

        t += distance / sqrt(deriv.x * deriv.x + deriv.y * deriv.y);
    }

    vel = 0.00001;
    last_angular_vel = 0;
    angular_accel = 0;
    t = 1;

    while (t >= 0)
    {
        auto p1 = path.valueAt(t);

        deriv = path.derivative().valueAt(t);
        derivSecond = path.derivative().derivative().valueAt(t);

        theta = std::atan2(deriv.y, deriv.x);

        curvature = path.curvatureAt(deriv, derivSecond);

        double maxSpeed = constraints->maxSpeed(curvature);

        angular_vel = vel * curvature;
        angular_accel = (angular_vel - last_angular_vel) / dt;
        last_angular_vel = angular_vel;
        max_accel = this->constraints->max_dec - fabs(angular_accel * this->constraints->track_width / 2);

        double maxAchievable = vel + max_accel * dt;

        vel = std::min(maxSpeed, maxAchievable);

        double angular = vel*curvature;

        backwardPass.push_back(ProfilePoint(p1.x, p1.y, theta, curvature, t, vel, angular));

        double distance = vel * dt + 0.5 * max_accel * dt * dt;

        t -= distance / sqrt(deriv.x * deriv.x + deriv.y * deriv.y);
    }

    // Get lower of the two velocities at each point and store in trajectory
    for (int i = 0; i < backwardPass.size(); ++i)
    {
        auto forward = forwardPass[i];
        auto backward = backwardPass[backwardPass.size() - i - 1];
        auto vel = std::min(forward.vel, backward.vel);
        auto avel = vel*forward.curvature;

        this->profile.push_back(ProfilePoint(forward.x, forward.y, forward.theta, forward.curvature, forward.t, vel, avel));
    }
    
    for (auto p : getProfile()) {
		std::cout << p.x << ", " << p.y << ", " << p.theta << ", " << p.vel << ", " << (p.accel) << std::endl;
	}
    //? Removing this loop slows down code????
}