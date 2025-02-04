#pragma once

#include "sensorModel.h"
#include "utils.h"

#include "Eigen/Dense"
#include "pros/distance.hpp"
#include "units/units.hpp"
#include "config.h"

namespace loco {
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

    /**
     * @brief Sensor model representation of distance sensors pointed directly at the walls on a specified position on the robot.
     *
     * Uses a field model made up of 4 walls, represented by horizontal and vertical lines, and uses secant to predict the
     * distance to the wall and compares this against the measured value.
     */
    class DistanceSensorModel : public SensorModel {
    private:
        Eigen::Vector3f sensorOffset;
        pros::Distance distance;

        QLength measured = 0.0;
        bool exit = false;
        QLength std = 0.0;

    public:
        /**
         *
         * @param sensor_offset [x, y, ø] of the distance sensor relative to the tracking center of the robot.
         * @param distance pros::Distance object, moved to this object.
         */
        DistanceSensorModel(Eigen::Vector3f sensor_offset, pros::Distance distance)
            : sensorOffset(std::move(sensor_offset)),
              distance(std::move(distance)) {
        }

        /**
         * Update sensor reading
         */
        void update() override {
            const auto measuredMM = distance.get();

            exit = measuredMM == 9999 || distance.get_object_size() < 70;

            measured = measuredMM * millimetre;

            auto confidence = distance.get_confidence();

            if (confidence == 0) {
                std = 0.0;
                return;
            }

            std = 0.20 * measured / (confidence / 64.0);
        }

        /**
         * @brief Determine p(z, x) where z is the current distance sensor position, and x is the predicted position of the
         * robot.
         *
         * @param X The particle position
         * @return probability for the current distance sensor reading, given the robot is at the point X
         */
        [[nodiscard]] std::optional<double> p(const Eigen::Vector3f &X) override {
            if (exit) {
                return std::nullopt;
            }

            // std::cout << this <<" before: " << X.x() << ", " << X.y() << std::endl;

            auto angle = float(fmod(X.z() + sensorOffset.z(), 2*M_PI));

            Eigen::Vector2f x = X.head<2>() + Eigen::Rotation2Df(X.z()) * sensorOffset.head<2>();

            // std::cout << "after: "  << x.x() << ", " << x.y() << std::endl;

            float predicted = 50.0f;
            float theta = 0;

            if (theta = abs(angle - 0); theta < M_PI_2) {
                predicted = std::min((WALL_0_X - x.x()) / abs(cos(theta)), predicted);
            }

            if (theta = abs(angle - M_PI_2); theta < M_PI_2) {
                predicted = std::min((WALL_1_Y - x.y()) / abs(cos(theta)), predicted);
            }

            if (theta = abs(angle - M_PI); theta < M_PI_2) {
                predicted = std::min((x.x() - WALL_2_X) / abs(cos(theta)), predicted);
            }

            if (theta = abs(angle - (3*M_PI)/2); theta < M_PI_2) {
                predicted = std::min((x.y() - WALL_3_Y) / abs(cos(theta)), predicted);
            }

            if (theta = abs(angle - 2*M_PI); theta < M_PI_2) {
                predicted = std::min((WALL_0_X - x.x()) / abs(cos(theta)), predicted);
            }
            
            return cheap_norm_pdf((predicted - measured.getValue()) / std.getValue()) * LOCO_CONFIG::DISTANCE_WEIGHT;
        }

        ~DistanceSensorModel() override = default;
    };
}
