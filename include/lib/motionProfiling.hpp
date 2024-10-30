#include "Eigen/Dense"
#include "pose.hpp"
#include "bezier.h"

namespace lib {
    class Point2D
    {
    public:
        Point2D(double x, double y)
        {
            this->x = x;
            this->y = y;
        }
        Point2D()
        {
            this->x = 0;
            this->y = 0;
        }
        double x;
        double y;
    };

    class Constraints
    {
    public:
        Constraints(double max_vel, double max_acc, double friction_coef, double max_dec, double max_jerk, double track_width);
        double maxSpeed(double curvature);
        std::pair<double, double> wheelSpeeds(double angularVel, double vel);
        double max_vel;
        double max_acc;
        double friction_coef;
        double max_dec;
        double max_jerk;
        double track_width;
    };

    class ProfilePoint
    {
    public:
        ProfilePoint(double x, double y, double theta, double curvature, double t, double vel, double accel);
        ProfilePoint(double dist, double vel);
        double x;
        double y;
        double theta;
        double curvature;
        double t;
        double vel;
        double accel;
        double dist;

        friend std::ostream& operator<<(std::ostream& os, const ProfilePoint& point);
    };

    class ProfileGenerator
    {
    public:
        ProfileGenerator(Constraints *constraints, double dd);
        void generateProfile(bezier::Bezier<3> path);
        auto getProfile() { return profile; }

    private:
        Constraints *constraints;
        std::vector<ProfilePoint> profile;
        double dd;
        double duration;
    };
}