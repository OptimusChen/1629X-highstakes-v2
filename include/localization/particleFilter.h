#pragma once

#include "Eigen/Eigen"
#include "units/units.hpp"
#include "sensorModel.h"

#include <random>
#include <algorithm>
#include <iostream>

#include "config.h"
#include "lib/logging.hpp"

namespace loco
{
    /**
     * @brief Initializes a particle filter with a pre-specified number of particles.
     *
     * @warning Due to current efficiency limitations, the particle limit is 500 (Takes approximately 6ms to compute each
     * frame). For calculating frame time, estimate processing time to be 12µs/particle.
     *
     * @tparam L Number of particle to initialize the filter with. More is generally better for accuracy, however there are
     * diminishing returns once the particle count is greater than 100, view warning for notes on large particle quantities.
     */
    template <size_t L>
    class ParticleFilter
    {
        // Ensure particles are less than the max of 500 particles
        static_assert(std::less_equal<size_t>()(L, 500));

    private:
        // Particle filter variables
        std::array<std::array<float, 2>, L> particles;
        std::array<std::array<float, 2>, L> oldParticles;

        Eigen::Vector3f prediction{};

        std::vector<SensorModel *> sensors;

        QLength distanceSinceUpdate = 0.0;
        QTime lastUpdateTime = 0.0;

        const QLength minDistanceSinceUpdate = 1_in;
        const QTime maxUpdateInterval = 2_s;
        pros::Mutex predmutex;

        std::function<Angle()> angleFunction;
        std::ranlux24_base de;

        std::uniform_real_distribution<> fieldDist{-1.78308, 1.78308};

        bool noiseNextUpdate = false;
        bool addNoise = true;

        /**
         * @brief Returns the number of unique particles in the particle filter.
         * @return The number of unique particles in the particle filter.
         */
        uint32_t numUniqueParticles()
        {
            uint32_t unique = 0;

            for (size_t i = 1; i < L; i++)
            {
                const auto current = particles[i];
                const auto last = particles[i - 1];

                if (current[0] != last[0] || current[1] != last[1])
                {
                    unique++;
                }
            }

            return unique;
        }

        /**
         * @brief Resamples the particles based on the average weight of the particles.
         * @param avgWeight The average weight of the particles.
         */
        void resample(double avgWeight)
        {
            std::uniform_real_distribution distribution(0.0, avgWeight);
            const double randWeight = distribution(de);

            for (size_t i = 0; i < particles.size(); i++)
            {
                oldParticles[i] = particles[i];
            }

            size_t j = 0;
            auto cumulativeWeight = 0.0;

            for (size_t i = 0; i < L; i++)
            {
                const auto weight = static_cast<double>(i) * avgWeight + randWeight;

                while (cumulativeWeight < weight)
                {
                    if (j >= weights.size())
                    {
                        break;
                    }
                    cumulativeWeight += weights[j];
                    j++;
                }

                particles[i][0] = oldParticles[j - 1][0];
                particles[i][1] = oldParticles[j - 1][1];
            }
        }

    public:
        std::array<float, L> weights;

        /**
         * @brief Initializes the particle filter with the given angle function.
         * @param angle_function The function to get the angle of the robot.
         */
        explicit ParticleFilter(std::function<Angle()> angle_function)
            : angleFunction(std::move(angle_function))
        {
            for (auto &&particle : particles)
            {
                particle[0] = 0.0;
                particle[1] = 0.0;
            }
        }

        /**
         * @brief Gets the prediction of the particle filter.
         * @return The prediction of the particle filter.
         */
        Eigen::Vector3f getPrediction()
        {
            predmutex.take();
            auto a = prediction;
            predmutex.give();
            return a;
        }

        /**
         * @brief Gets the particles of the particle filter.
         * @return The particles of the particle filter.
         */
        std::array<Eigen::Vector3f, L> getParticles()
        {
            std::array<Eigen::Vector3f, L> particles;

            const Angle angle = angleFunction();

            for (size_t i = 0; i < L; i++)
            {
                particles[i] = Eigen::Vector3f(this->particles[i][0], this->particles[i][1], angle.getValue());
            }

            return particles;
        }

        /**
         * @brief Gets the particle at the given index.
         * @param i The index of the particle.
         * @return The particle at the given index.
         */
        Eigen::Vector3f getParticle(size_t i)
        {
            return {this->particles[i][0], this->particles[i][1], angleFunction().getValue()};
        }

        /**
         * @brief Updates the particle filter with the given prediction function and current time.
         * @param predictionFunction The function to predict the next movement of the robot.
         * @param now The current time.
         */
        void update(const std::function<Eigen::Vector2f()> &predictionFunction, QTime now)
        {
            // If the angle is not finite, return
            if (!isfinite(angleFunction().getValue()))
            {
                return;
            }

            const Angle angle = angleFunction();

            /*
             * Shift the particles by the prediction function and update the distance since the last update. If the distance
             * since the last update is less than the minimum distance since the last update and the maximum update interval
             * has not been reached, return.
             */
            for (auto &&particle : particles)
            {
                auto prediction = predictionFunction();
                particle[0] += prediction.x();
                particle[1] += prediction.y();
            }

            distanceSinceUpdate += predictionFunction().norm();

            if (distanceSinceUpdate < minDistanceSinceUpdate && maxUpdateInterval > now)
            {
                return;
            }

            // Update the sensors
            for (auto &&sensor : this->sensors)
            {
                sensor->update();
            }

            /*
             * Calculate the weight of each particle by multiplying the weight of each sensor. If the total weight is 0, return.
             */
            double totalWeight = 0.0;

            for (size_t i = 0; i < L; i++)
            {
                weights[i] = 1.0;

                if (outOfField(particles[i]))
                {
                    particles[i][0] = fieldDist(de);
                    particles[i][1] = fieldDist(de);
                }

                auto particle = Eigen::Vector3f(particles[i][0], particles[i][1], angle.getValue());

                for (const auto sensor : sensors)
                {
                    if (auto weight = sensor->p(particle); weight.has_value() && isfinite(weight.value()))
                    {
                        weights[i] = weights[i] * weight.value();
                    }
                }

                totalWeight = totalWeight + weights[i];
            }

            const double avgWeight = totalWeight / static_cast<double>(L);

            // Compute `standard deviation
            float variance = 0.0f;
            for (const auto &weight : weights)
            {
                variance += (weight - avgWeight) * (weight - avgWeight);
            }
            variance /= static_cast<float>(L);
            float standardDeviation = std::sqrt(variance) / avgWeight;
            size_t uniqueParticles = numUniqueParticles();

            pfLogger.push_log(LogType::DEVIATION_AND_UNIQUE, {standardDeviation, static_cast<float>(uniqueParticles), -1, -1});

            if (standardDeviation > 0.15)
            {
                resample(avgWeight);
            }

            if (uniqueParticles < 40 && addNoise) {
                /*
                 * If the number of unique particles is less than 5, add noise to the particles to prevent the filter from
                 * converging to a single point.
                 */
                std::uniform_real_distribution noise(-0.127, 0.127);

                for (size_t i = 0; i < L; i++)
                {
                    particles[i][0] += noise(de);
                    particles[i][1] += noise(de);
                }
            }

            if (noiseNextUpdate)
            {
                initNormal({prediction.x(), prediction.y()}, Eigen::Matrix2f::Identity() * 0.1, false);

                noiseNextUpdate = false;
            }

            // Calculate the average x and y position of the particles
            float xSum = 0.0, ySum = 0.0;
            for (size_t i = 0; i < L; i++)
            {
                xSum += particles[i][0];
                ySum += particles[i][1];
            }

            // Update the prediction and reset the distance since the last update
            predmutex.take();
            prediction = Eigen::Vector3f(xSum / static_cast<float>(L), ySum / static_cast<float>(L), angle.getValue());
            predmutex.give();

            lastUpdateTime = now;
            distanceSinceUpdate = 0.0;
        }

        /**
         * @brief Initializes the particle filter with a normal distribution.
         * @param mean The mean of the normal distribution.
         * @param covariance The covariance of the normal distribution.
         * @param flip Whether to flip the y-axis.
         */
        void initNormal(const Eigen::Vector2f &mean, const Eigen::Matrix2f &covariance, const bool flip)
        {
            for (auto &&particle : this->particles)
            {
                Eigen::Vector2f p = mean + covariance * Eigen::Vector2f::Random();
                particle[0] = p.x();
                particle[1] = p.y() * (flip ? -1.0 : 1.0);
            }

            prediction.z() = angleFunction().getValue();
            distanceSinceUpdate += 2.0 * distanceSinceUpdate;
        }

        /**
         * @brief Checks if the given vector is out of the field.
         * @param vector The vector to check.
         * @return Whether the vector is out of the field.
         */
        static bool outOfField(const std::array<float, 2> &vector)
        {
            const float wall = 1.78308;
            return vector[0] > wall || vector[0] < -wall || vector[1] < -wall || vector[1] > wall;
        }

        /**
         * @brief Initializes the particle filter with a uniform distribution.
         * @param minX The minimum x value.
         * @param minY The minimum y value.
         * @param maxX The maximum x value.
         * @param maxY The maximum y value.
         */
        void initUniform(const QLength minX, const QLength minY, const QLength maxX, const QLength maxY)
        {
            std::uniform_real_distribution xDistribution(minX.getValue(), maxX.getValue());
            std::uniform_real_distribution yDistribution(minY.getValue(), maxY.getValue());

            for (auto &&particle : this->particles)
            {
                particle[0] = xDistribution(de);
                particle[1] = yDistribution(de);
            }
        }

        /**
         * @brief Gets the sensors used by the particle filter.
         * @return The sensors used by the particle filter.
         */
        const std::vector<SensorModel *> &getSensors() const
        {
            return sensors;
        }

        /**
         * @brief Adds a sensor to the particle filter.
         * @param sensor The sensor to add.
         */
        void addSensor(SensorModel *sensor)
        {
            this->sensors.emplace_back(sensor);
        }

        /**
         * @brief Prints the sensors used by the particle filter.
         */
        void printsensors()
        {
            for (auto sensor : sensors)
            {
                std::cout << sensor << std::endl;
            }
        }

        /**
         * @brief Sets the noise for the next update.
         * @param noise Whether to add noise to the next update.
         */
        void setNoiseNextUpdate(bool noise)
        {
            noiseNextUpdate = noise;
        }

        /**
         * @brief Sets whether to add noise to the particle filter.
         * @param noise Whether to add noise to the particle filter.
         */
        void setAddNoise(bool noise)
        {
            addNoise = noise;
        }

        /**
         * @brief Gets the angle function used by the particle filter.
         * @return The angle function used by the particle filter.
         */
        Angle getAngle()
        {
            return angleFunction();
        }
    };
}