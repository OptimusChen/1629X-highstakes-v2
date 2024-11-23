#include "autonomous.hpp"
#include "lib/util.hpp"

void bestautonfr::skills(Robot robot) {
    // robot.turnToHeading(180, 500);
    // robot.turnToHeading(90, 500);
    
    // robot.set_pose_mode(MCL);

    // left
    robot.ramsete({{0, 0}, {0, 0.5}, {0.5, 0}, {0.5, 0.5}}, 1, true);
    // right
    // robot.ramsete({{0, 0}, {0, 0.5}, {0, 0.5}, {0.5, 0.5}}, 0.5, true);
    // robot.moveToPoint(0, 24, 2000);

    return;

    // robot.turnToHeading(0, 500);

    // robot.ramsete({{0, 0}, {0, 1.2}, {1.2, 0}, {1.2, 1.2}}, 0.5, true);
    // robot.ramsete({{1.2, 1.2}, {1.2, 0}, {0, 1.2}, {0, 0}}, 0.5, false);

    return;

    robot.ramsete({{-1.543, -0.018}, {-1.071, 0.048}, {-1.21, 0}, {-1.21, -0.672}}, 1, false);

    delay(500);

    // robot.set_mogo(true);

    robot.ramsete({{-1.213, -0.68}, {-0.753, -0.563}, {-0.481, -0.272}, {0.1, -1.507}});


    // 
    // , {0.596, -1.186}, {-0.081, -1.888}, {-0.414, -1.198}, {-0.626, -1.198}, {-0.626, -1.198}, {-0.947, -1.186}, {-1.101, -1.195}, {-1.576, -1.192}

    robot.set_pose_mode(MCL);

    delay(3000);

    // robot.ramsete({{-1.21, -0.472}, {-1.21, 0}, {-1.071, 0.048}, {-1.543, -0.018}}, true);
}   