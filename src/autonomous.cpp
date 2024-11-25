#include "autonomous.hpp"
#include "lib/util.hpp"

void bestautonfr::skills(Robot robot) {
    robot.ramsete({{0, 0}, {0, 40}, {40, 0}, {40, 40}});
    return;

    robot.moveToPoint(-50, 0, 1000);
    robot.turnToHeading(90, 500);
    robot.moveToPoint(-50, -23, 1000, false);

    robot.turnToHeading(0, 500);

    robot.intake(false);

    robot.ramsete({
        {-46.864, -23.636}, {-30.755, -13.971}, {-16.162, -26.858}, {0.326, -47.516},
        {0.326, -47.516}, {16.814, -68.173}, {35.576, -40.314}, {23.447, -23.636}
    });

    return;
}   