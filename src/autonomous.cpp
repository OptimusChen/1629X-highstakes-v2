#include "autonomous.hpp"
#include "lib/util.hpp"

MPConstraint fast{60, 150, INCH};
MPConstraint slow{30, 150, INCH};

void bestautonfr::skills(Robot* robot) {
    // robot->ramsete({{0, 0}, {0, 60}, {0, 0}, {0, 60}}, fast);
    // robot->ramsete({{0, 0}, {0, 40}, {40, 0}, {40, 40}}, fast);
    // return;

    robot->set_pose_mode(MCL);

    pros::delay(3000);

    robot->moveToPoint(-50, 0, 1000);
    robot->turnToHeading(90, 1000);

    robot->moveToPoint(-47, -23, 1000, false);

    robot->turnToHeading(0, 1000);

    // robot->turnToHeading(270, 100);
    

    // robot->moveToPoint(-50, 0, 1000);
    // robot->turnToHeading(0, 1000);
    // robot->moveToPoint(-60, 0, 1000, false);  


    // return;

    robot->intake(false);

    // robot->set_pose_mode(MCL);

    // robot->ramsete({
    //     {-47.337, -23.731}, {-21.563, -24.489}, {-20.237, -18.993}, {-15.309, -35.292}
    // }, fast);

    // robot->ramsete({
    //     {-15.309, -35.292}, {-10.382, -51.59}, {-1.853, -44.389}, {11.413, -47.421}
    // }, fast);

    // return;

    // robot->ramsete({
    //     {11.413, -47.421}, {24.679, -50.453}, {12.929, -61.445}, {0.042, -59.929}
    // }, fast);

    // robot->ramsete({
    //     {0.042, -59.929}, {-12.845, -58.413}, {6.675, -44.768}, {-60.414, -46.473}
    // }, fast);

    // return;

    robot->ramsete({
        {-47.337, -23.731}, {-21.563, -24.489}, {-20.237, -18.993}, {-15.309, -35.292},
        {-15.309, -35.292}, {-10.382, -51.59}, {-1.853, -44.389}, {11.413, -47.421},
        {11.413, -47.421}, {24.679, -50.453}, {12.644, -59.265}, {-0.243, -57.749},
        {0.042, -59.929}, {-12.845, -58.413}, {6.675, -44.768}, {-60.414, -46.473}
    }, fast);

    return;
}   