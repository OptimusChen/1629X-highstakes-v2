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

void ProfileGenerator::generateProfile(bezier::BezierSpline<3> path)
{
    this->profile.clear();

    // dont ask    
    double dt = 0.01;
    double vel = 0;

    std::vector<ProfilePoint> forwardPass;
    std::vector<ProfilePoint> backwardPass;

    double localT = 0;
    int segmentIndex = 0;

    double curvature;
    double angular_vel = 0;
    double angular_accel = 0;
    double last_angular_vel = 0;
    double max_accel = 0;
    double theta = 0;
    bezier::Point deriv;
    bezier::Point derivSecond;

    std::cout << path.segments[0].valueAt(1).x << ", " << path.segments[0].valueAt(1).y << std::endl;

    int i = 0;

    while (true)
    {
        if (segmentIndex == path.segments.size()) {
            // std::cout << "breaking: " << forwardPass.at(forwardPass.size()-1).t << std::endl;
            break;
        }

        auto segment = path.segments[segmentIndex];

        auto p1 = segment.valueAt(localT);

        deriv = segment.derivative().valueAt(localT);
        derivSecond = segment.derivative().derivative().valueAt(localT);

        theta = std::atan2(deriv.y, deriv.x);

        curvature = segment.curvatureAt(deriv, derivSecond);

        double maxSpeed = constraints->maxSpeed(curvature);

        angular_vel = vel * curvature;
        angular_accel = (angular_vel - last_angular_vel) / dt;
        last_angular_vel = angular_vel;
        max_accel = this->constraints->max_acc - fabs(angular_accel * this->constraints->track_width / 2);

        max_accel = this->constraints->max_acc;

        double maxAchievable = vel + dt * max_accel;

        vel = std::min(maxSpeed, maxAchievable);

        double angular = vel*curvature;

        forwardPass.push_back(ProfilePoint(p1.x, p1.y, theta, curvature, segmentIndex + localT, vel, angular));

        double distance = vel * dt + 0.5 * max_accel * dt * dt;

        localT += distance / sqrt(deriv.x * deriv.x + deriv.y * deriv.y);

        if (localT > 1) {
            segmentIndex++;
            localT = 0;
        }

        // std::cout << segmentIndex + localT << std::endl;
    }

    vel = 0;
    last_angular_vel = 0;
    angular_accel = 0;
    localT = 1;
    segmentIndex = path.segments.size() - 1;

    while (true)
    {
        if (segmentIndex == -1) break;

        auto segment = path.segments[segmentIndex];

        auto p1 = segment.valueAt(localT);

        deriv = segment.derivative().valueAt(localT);
        derivSecond = segment.derivative().derivative().valueAt(localT);

        theta = std::atan2(deriv.y, deriv.x);

        curvature = segment.curvatureAt(deriv, derivSecond);

        double maxSpeed = constraints->maxSpeed(curvature);

        angular_vel = vel * curvature;
        angular_accel = (angular_vel - last_angular_vel) / dt;
        last_angular_vel = angular_vel;
        max_accel = this->constraints->max_dec - fabs(angular_accel * this->constraints->track_width / 2);

        max_accel = this->constraints->max_dec;

        double maxAchievable = vel + max_accel * dt;

        vel = std::min(maxSpeed, maxAchievable);

        double angular = vel*curvature;

        backwardPass.push_back(ProfilePoint(p1.x, p1.y, theta, curvature, segmentIndex + localT, vel, angular));

        double distance = vel * dt + 0.5 * max_accel * dt * dt;

        localT -= distance / sqrt(deriv.x * deriv.x + deriv.y * deriv.y);

        if (localT < 0) {
            segmentIndex--;
            localT = 1;
        }
    }

    std::cout << backwardPass.size() << " " << forwardPass.size() << std::endl;

    // Get lower of the two velocities at each point and store in trajectory
    for (int i = 0; i < std::min(forwardPass.size(), backwardPass.size()); ++i)
    {
        auto forward = forwardPass[i];
        auto backward = backwardPass[backwardPass.size() - i - 1];
        auto vel = std::min(forward.vel, backward.vel);

        this->profile.push_back(ProfilePoint(forward.x, forward.y, forward.theta, forward.curvature, backward.curvature, vel, max_accel));
    }
    
    for (auto p : getProfile()) {
		std::cout << p.x << ", " << p.y << ", " << p.curvature << ", " << p.t << ", " << (p.vel*p.curvature) << ", " << (p.t) << std::endl;
	}
    //? Removing this loop slows down code????
}