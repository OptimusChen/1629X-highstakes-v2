#include "autonomous.hpp"

void bestautonfr::skills(Robot robot) {
    // robot.set_arm_pistons(true);

    // robot.ramsete({{0, 1.546}, {0.04, 1.147}, {-0.364, 1.245}, {-0.518, 1.216}});

    robot.ramsete({{0,0},{0,1},{1,0},{1,1}},true);

    robot.set_mogo(true);
}   