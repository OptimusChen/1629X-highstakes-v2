#pragma once

#include "distance.hpp"
#include "pose.hpp"
#include <vector>
#include <random>
#include <optional>

namespace lib::mcl {
    class MCL {
    public:
        // REMEMBER THIS TAKES IN METERS
        MCL(DistanceModel* distance_model, size_t num_particles, Pose initial_pose, float sensor_std_dev)
            : distance_model(distance_model), num_particles(num_particles), sensor_std_dev(sensor_std_dev) {
            
            // Initialize particles around the initial pose
            std::default_random_engine generator;
            std::normal_distribution<float> dist_x(initial_pose.x, sensor_std_dev);
            std::normal_distribution<float> dist_y(initial_pose.y, sensor_std_dev);
            std::normal_distribution<float> dist_theta(initial_pose.theta, sensor_std_dev);

            particles.reserve(num_particles);
            for (size_t i = 0; i < num_particles; ++i) {
                particles.emplace_back(dist_x(generator), dist_y(generator), dist_theta(generator));
                weights.push_back(1.0 / num_particles);  // Initialize with equal weight
            }
        }

        void update(const Pose& control_movement) {
            // Prediction Step - Move each particle based on control movement and add random noise
            std::default_random_engine generator;
            std::normal_distribution<float> noise_dist(0, sensor_std_dev);
            for (auto& particle : particles) {
                particle.x += control_movement.x + noise_dist(generator);
                particle.y += control_movement.y + noise_dist(generator);
                particle.theta += control_movement.theta + noise_dist(generator);
            }

            // Measurement Update - Update weights based on the sensor model's probability
            distance_model->update();  // Update the sensor model with the latest data
            for (size_t i = 0; i < num_particles; ++i) {
                std::optional<double> probability = distance_model->probability(particles[i]);
                weights[i] = probability ? *probability : 0.0;
            }

            normalize_weights();  // Normalize the weights for resampling

            // Resampling Step - Resample particles based on their weights
            resample_particles();
        }

        Pose get_estimate() const {
            // Calculate the weighted average pose as the estimated position
            Pose estimate{0.0, 0.0, 0.0};
            double total_weight = 0.0;

            for (size_t i = 0; i < num_particles; ++i) {
                estimate.x += particles[i].x * weights[i];
                estimate.y += particles[i].y * weights[i];
                estimate.theta += particles[i].theta * weights[i];
                total_weight += weights[i];
            }

            if (total_weight > 0) {
                estimate.x /= total_weight;
                estimate.y /= total_weight;
                estimate.theta /= total_weight;
            }

            return estimate;
        }

    private:
        DistanceModel* distance_model;
        size_t num_particles;
        float sensor_std_dev;

        std::vector<Pose> particles;
        std::vector<double> weights;

        void normalize_weights() {
            double sum = 0.0;
            for (double weight : weights) {
                sum += weight;
            }
            if (sum > 0) {
                for (double& weight : weights) {
                    weight /= sum;
                }
            }
        }

        void resample_particles() {
            std::vector<Pose> new_particles;
            new_particles.reserve(num_particles);

            std::default_random_engine generator;
            std::discrete_distribution<size_t> distribution(weights.begin(), weights.end());

            for (size_t i = 0; i < num_particles; ++i) {
                new_particles.push_back(particles[distribution(generator)]);
            }

            particles = std::move(new_particles);
        }
    };
}
