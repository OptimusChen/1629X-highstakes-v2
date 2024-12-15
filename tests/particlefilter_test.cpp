#include <gtest/gtest.h>
#include "localization/particleFilter.h"
#include "localization/distance.h"
#include "pros/distance.hpp"
#include "pros/rtos.h"
#include "mockpros/mock_hardware.hpp"

using namespace loco;
using namespace prosMock;

/**
 * Simulate robot angle.
 * @{
 */
static Angle angle = 0_deg;
static Angle get_robot_angle()
{
    return angle;
}
/**@} */

class ParticleFilterTest : public ::testing::Test
{
protected:
    /** The particle filter. */
    static constexpr size_t PARTICLE_COUNT = 50;
    ParticleFilter<PARTICLE_COUNT> pf{get_robot_angle};

    /** Three distance sensors. */
    static constexpr uint8_t L_DISTANCE = 8u;
    static constexpr uint8_t R_DISTANCE = 4u;
    static constexpr uint8_t B_DISTANCE = 10u;

    pros::Distance left_dist{L_DISTANCE};
    pros::Distance right_dist{R_DISTANCE};
    pros::Distance back_dist{B_DISTANCE};
    DistanceSensorModel rightDistance{Eigen::Vector3f((-3.75_in).getValue(), (-5.75_in).getValue(), (270_deg).getValue()), right_dist};
    DistanceSensorModel leftDistance{Eigen::Vector3f((-3.25_in).getValue(), (5.75_in).getValue(), (90_deg).getValue()), left_dist};
    DistanceSensorModel backDistance{Eigen::Vector3f((-4.375_in).getValue(), (5.25_in).getValue(), (180_deg).getValue()), back_dist};

    void SetUp() override
    {
        // Initialize the particle filter with 3 distance sensors.

        pf.addSensor(&rightDistance);
        pf.addSensor(&leftDistance);
        pf.addSensor(&backDistance);

        EXPECT_EQ(pf.getSensors().size(), 3);
        EXPECT_EQ(pf.getSensors()[0], &rightDistance);
        EXPECT_EQ(pf.getSensors()[1], &leftDistance);
        EXPECT_EQ(pf.getSensors()[2], &backDistance);
    }
};

TEST_F(ParticleFilterTest, InitUniform)
{
    EXPECT_EQ(pf.getParticles().size(), PARTICLE_COUNT);

    // Test that particles are initialized within the specified bounds
    pf.initUniform(0, 0, 1, 1);
    for (const auto &particle : pf.getParticles())
    {
        EXPECT_GE(particle[0], 0);
        EXPECT_LE(particle[0], 1);
        EXPECT_GE(particle[1], 0);
        EXPECT_LE(particle[1], 1);
    }
}

TEST_F(ParticleFilterTest, OutOfField)
{
    // Test the outOfField static method
    EXPECT_TRUE(pf.outOfField({2.0, 0.0}));
    EXPECT_FALSE(pf.outOfField({0.0, 0.0}));
}

TEST_F(ParticleFilterTest, getAngle)
{
    EXPECT_EQ(pf.getAngle().getValue(), 0.f);

    // Set Angle and validate.
    angle = 90_deg;
    EXPECT_EQ(pf.getAngle().getValue(), 0.5_pi);

    angle = 45_deg;
    EXPECT_EQ(pf.getAngle().getValue(), 0.25_pi);

    angle = 135_deg;
    EXPECT_EQ(pf.getAngle().getValue(), 0.75_pi);
}

TEST_F(ParticleFilterTest, getPrediction)
{
    // Test getPrediction method
    EXPECT_EQ(pf.getPrediction(), Eigen::Vector3f(0, 0, 0));

    // Move straight with speed 1 m/s.
    constexpr auto speed_mps = 1.0f;
    // update Interval in milliseconds
    constexpr auto update_interval_ms = 10u;
    for (uint32_t t_ms = 0; t_ms < 100; t_ms += update_interval_ms)
    {
        pf.update([speed_mps]()
                  { return Eigen::Rotation2Df(0.f) * Eigen::Vector2f({speed_mps, 0.0}); }, pros::millis() * millisecond);

        // distance sensor update.
        advance_time_ms(update_interval_ms);
    }
    EXPECT_EQ(pf.getPrediction(), Eigen::Vector3f(0.1f, 0, 0));
}

TEST_F(ParticleFilterTest, zeroSensorConfidence)
{
    // Test distance sensor with zero confidence.
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}