#include "autonomous.hpp"
#include "lib/util.hpp"

MPConstraint fast{60, 150, INCH};
MPConstraint slow{30, 120, INCH};

void bestautonfr::skills(Robot* robot) {
    // return;
    // robot->ramsete({
    //     {0, 0}, {0, 47}, {47, 0}, {47, 47}
    //     // {40, 40}, {40, 80}, {-21, 52}, {0, 0}
    // }, slow);
    // return;
    // robot->ramsete({
    //     {47, 47}, {47, 0}, {0, 47}, {0, 0}
    // }, slow, false);
    // return;
    // robot->turnToHeading(270, 1000);
    // robot->ramsete({
    //     {-60, 0}, {-60, -20}, {-20, 0}, {-23, -23}
    // }, fast);
    // return;

    robot->moveToPoint(-50, 0, 1000);
    robot->turnToHeading(90, 1000);

    robot->moveToPoint(-50, -23, 1000, false);

    robot->turnToHeading(0, 1000);
    // robot->turnToHeading(270, 100);
    

    // robot->moveToPoint(-50, 0, 1000);
    // robot->turnToHeading(0, 1000);
    // robot->moveToPoint(-60, 0, 1000, false);  


    // return;

    // robot.intake(false);

    robot->set_pose_mode(MCL);

    // robot->ramsete({
    //     {-46.674, -23.636}, {-22.416, -17.761}, {18.561, -37.85}, {11.981, -51.116}
    //     // {11.981, -51.116}, {0.326, -74.616}, {-2.233, -37.85}, {-60.793, -47.705},
    //     // {-60.793, -47.705}, {-69.149, -49.111}, {-62.783, -70.731}, {-46.579, -57.749}
    // }, fast);

    return;
}   