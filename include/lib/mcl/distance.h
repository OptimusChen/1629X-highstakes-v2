#include "Eigen/Dense"
#include "pros/distance.hpp"
#include "../pose.hpp"
#include "../util.hpp"

const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > WALLS = {
    {{1.78308, 1.78308}, {1.78308, -1.78308}},
    {{1.78308, -1.78308}, {-1.78308, -1.78308}},
    {{-1.78308, -1.78308}, {-1.78308, 1.78308}},
    {{-1.78308, 1.78308}, {1.78308, 1.78308}},
};

constexpr float WALL_0_X = 1.78308;
constexpr float WALL_1_Y = 1.78308;
constexpr float WALL_2_X = -1.78308;
constexpr float WALL_3_Y = -1.78308;

#define DISTANCE_WEIGHT 69420

namespace lib::mcl {
    class DistanceModel {
        public:
            pros::Distance* sensor;
            Pose offset;

            double measured;
            double standard;
            bool exit = false;

            DistanceModel(pros::Distance* sensor, Pose offset) {
                this->sensor = sensor;
                this->offset = offset;
            }

            void update() {
                int measuredMM = sensor->get();

                exit = measuredMM == 9999 || sensor->get_object_size() < 70;

                measured = measuredMM * 0.001;

                standard = 0.20 * measured / (sensor->get_confidence() / 64.0);
            }

            std::optional<double> probability(Pose actual) {
                if (exit) {
                    return std::nullopt;
                }

                actual = actual.meters();

                Eigen::Vector3f sensorOffset(offset.meters().x, offset.meters().y, offset.theta);
                Eigen::Vector3f X(actual.x, actual.y, actual.theta);

                auto angle = X.z() + sensorOffset.z();

                Eigen::Vector2f x = X.head<2>() + Eigen::Rotation2Df(X.z()) * sensorOffset.head<2>();

                float predicted = 50.0f;

                if (const auto theta = abs(std::remainder(0.0f, angle)); theta < M_PI_2) {
                    predicted = std::min((WALL_0_X - x.x()) / cos(theta), predicted);
                }

                if (const auto theta = abs(std::remainder(static_cast<float>(M_PI_2), angle)); theta < M_PI_2) {
                    predicted = std::min((WALL_1_Y - x.y()) / cos(theta), predicted);
                }

                if (const auto theta = abs(std::remainder(static_cast<float>(M_PI), angle)); theta < M_PI_2) {
                    predicted = std::min((x.x() - WALL_2_X) / cos(theta), predicted);
                }

                if (const auto theta = abs(std::remainder(static_cast<float>(M_3PI_4), angle)); theta < M_PI_2) {
                    predicted = std::min((x.y() - WALL_3_Y) / cos(theta), predicted);
                }

                return util::cheap_norm_pdf((predicted - measured) / standard) * DISTANCE_WEIGHT;
            }
    };
}