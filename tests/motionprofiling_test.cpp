#include <gtest/gtest.h>
#include "lib/motionProfiling.hpp"  // Include the header file for the class

using namespace lib;

class MotionProfilingTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize any shared resources for the tests
    }

    void TearDown() override {
        // Clean up any shared resources for the tests
    }

    // Add any member variables or helper functions here
};

TEST_F(MotionProfilingTest, TestProfileGeneration) {
    // Create an instance of the class
    ProfileGenerator pathGenerator(new Constraints(1.0, 1.0, 1.0, 1.0, 1.0, 1.0), 0.1);

    pathGenerator.generateProfile(bezier::BezierSpline<3>({{0, 0}, {0, 1}, {1, 0}, {1, 1}}));

    // Verify the output
    const auto& profile = motionProfiling.getProfile();
    ASSERT_FALSE(profile.empty());
    
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}