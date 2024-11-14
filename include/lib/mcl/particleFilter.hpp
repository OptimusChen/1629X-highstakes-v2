#include "distance.h"
#include "../units.hpp"

#include <random>
#include <algorithm>

#define L 500

namespace lib::mcl {
    class ParticleFilter {
    private:
        /**
         *
         */
        std::array<std::array<float, 2>, L> particles;
        std::array<std::array<float, 2>, L> oldParticles;
        std::array<float, L> weights;

        Eigen::Vector3f prediction{};

        std::vector<DistanceModel*> sensors;

        Length distanceSinceUpdate = 0.0_in;
        Time lastUpdateTime = 0.0_sec;

        Length maxDistanceSinceUpdate = 1_in;
        Time maxUpdateInterval = 2_sec;

        std::function<Angle()> angleFunction;
        std::ranlux24_base de;

        std::uniform_real_distribution<> fieldDist{-1.78308 / METERS, 1.78308 / METERS};

    public:
        explicit ParticleFilter(std::function<Angle()> angle_function)
            : angleFunction(std::move(angle_function)) {
            for (auto &&particle: particles) {
                particle[0] = 0.0;
                particle[1] = 0.0;
            }
        }

        ParticleFilter() : angleFunction([]() -> Angle { return Angle(0); }) {}

        Eigen::Vector3f getPrediction() {
            return prediction;
        }

        std::array<Eigen::Vector3f, L> getParticles() {
            std::array<Eigen::Vector3f, L> particles;

            const Angle angle = angleFunction();

            for (size_t i = 0; i < L; i++) {
                particles[i] = Eigen::Vector3f(this->particles[i][0], this->particles[i][1], angle.value);
            }

            return particles;
        }

        Eigen::Vector3f getParticle(size_t i) {
            return {this->particles[i][0], this->particles[i][1], angleFunction().value};
        }

        void update(const std::function<Eigen::Vector2f()> &predictionFunction) {
            if (!isfinite(angleFunction().value)) {
                return;
            }

            auto start = pros::micros();

            const Angle angle = angleFunction();

            for (auto &&particle: particles) {
                auto prediction = predictionFunction();
                particle[0] += prediction.x();
                particle[1] += prediction.y();
            }

            distanceSinceUpdate.value += predictionFunction().norm();

            if (distanceSinceUpdate < maxDistanceSinceUpdate && maxUpdateInterval > pros::millis() * 1000_sec) {
                return;
            }

            for (auto &&sensor: this->sensors) {
                sensor->update();
            }

            double totalWeight = 0.0;

            for (size_t i = 0; i < L; i++) {
                weights[i] = 1.0;

                if (outOfField(particles[i])) {
                    particles[i][0] = fieldDist(de);
                    particles[i][1] = fieldDist(de);
                }

                auto particle = Eigen::Vector3f(particles[i][0], particles[i][1], angle.value);

                for (const auto sensor: sensors) {
                    if (auto weight = sensor->probability(Pose(particle.x(), particle.y(), particle.z(), true)); weight.has_value() && isfinite(weight.value())) {
                        weights[i] = weights[i] * weight.value();
                    }
                }

                weights[i] = weights[i];

                totalWeight = totalWeight + weights[i];
            }

            if (totalWeight == 0.0) {
                std::cout << "Warning: Total weight equal to 0" << std::endl;
                return;
            }

            const double avgWeight = totalWeight / static_cast<double>(L);

            std::uniform_real_distribution distribution(0.0, avgWeight);
            const double randWeight = distribution(de);

            for (size_t i = 0; i < particles.size(); i++) {
                oldParticles[i] = particles[i];
            }

            size_t j = 0;
            auto cumulativeWeight = 0.0;

            float xSum = 0.0, ySum = 0.0;

            for (size_t i = 0; i < L; i++) {
                const auto weight = static_cast<double>(i) * avgWeight + randWeight;

                while (cumulativeWeight < weight) {
                    if (j >= weights.size()) {
                        break;
                    }
                    cumulativeWeight += weights[j];
                    j++;
                }

                particles[i][0] = oldParticles[j - 1][0];
                particles[i][1] = oldParticles[j - 1][1];

                xSum += particles[i][0];
                ySum += particles[i][1];
            }

            prediction = Eigen::Vector3f(xSum / static_cast<float>(L), ySum / static_cast<float>(L), angle.value);

            lastUpdateTime = pros::millis() * 1000_sec;
            distanceSinceUpdate = 0.0_in;
        }

        void initNormal(const Eigen::Vector2f &mean, const Eigen::Matrix2f &covariance, const bool flip) {
            for (auto &&particle: this->particles) {
                Eigen::Vector2f p = mean + covariance * Eigen::Vector2f::Random();
                particle[0] = p.x();
                particle[1] = p.y() * (flip ? -1.0 : 1.0);
            }

            prediction.z() = angleFunction().value;
            distanceSinceUpdate += 2.0 * distanceSinceUpdate;
        }

        static bool outOfField(const std::array<float, 2> &vector) {
            return vector[0] > 1.78308 / METERS || vector[0] < -1.78308 / METERS || vector[1] < -1.78308 / METERS || vector[1] > 1.78308 / METERS;
        }

        void initUniform(const Length minX, const Length minY, const Length maxX, const Length maxY) {
            std::uniform_real_distribution xDistribution(minX.value, maxX.value);
            std::uniform_real_distribution yDistribution(minY.value, maxY.value);

            for (auto &&particle: this->particles) {
                particle[0] = xDistribution(de);
                particle[1] = yDistribution(de);
            }
        }

        void addSensor(DistanceModel* sensor) {
            this->sensors.emplace_back(sensor);
        }

        Angle getAngle() {
            return angleFunction();
        }
    };
}