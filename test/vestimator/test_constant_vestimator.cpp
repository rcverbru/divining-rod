#include <gtest/gtest.h>
#include <vestimator/constant_vestimator.hpp>
#include <vestimator/imu_vestimator.hpp>
#include <vestimator/wheel_tick_vestimator.hpp>

class TestConstantVestimator : public ::testing::Test {
protected:
    diviner::Params<diviner::ConstantVestimatorParams, diviner::IVestimatorParams> params;
    std::shared_ptr<diviner::ConstantVestimator> constant_vestimator;
    std::vector<diviner::Velocity> velocities;
    geometry_msgs::TransformStamped transform_;
    std::vector<geometry_msgs::PoseStamped> veh_pose;

    void SetUp() override {
        // Initialize sample vehicle poses
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 2;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        veh_pose.push_back(pose);

        pose.pose.position.x = 1;
        veh_pose.push_back(pose);

        pose.pose.position.x = 0;
        veh_pose.push_back(pose);

        // Create the ConstantVestimator with the correct parameters type
        constant_vestimator = std::make_shared<diviner::ConstantVestimator>(params);

        // Set up TransformStamped
        transform_.transform.rotation.x = 0.0;
        transform_.transform.rotation.y = 0.0;
        transform_.transform.rotation.z = 0.0;
        transform_.transform.rotation.w = 1.0;
        transform_.transform.translation.x = 1.0;
    }

    void TearDown() override {
        if (constant_vestimator != nullptr) {
            constant_vestimator.reset();
        }
    }
};

TEST_F(TestConstantVestimator, initialization_without_segfault) {
    // Test that the estimator initializes without errors
}

TEST_F(TestConstantVestimator, test_linear_velocity_estimation) {
    // Run the estimate function
    constant_vestimator->estimate(velocities, transform_, veh_pose);

    // Expected velocity
    diviner::Velocity expected_vel;
    expected_vel.linear.x = 1.0; // Expected linear velocity along x-axis
    expected_vel.linear.y = 0.0;
    expected_vel.linear.z = 0.0;

    // Verify that the estimated velocity matches the expected
    ASSERT_EQ(velocities.size(), 1);
    EXPECT_NEAR(velocities[0].linear.x, expected_vel.linear.x, 1e-5);
    EXPECT_NEAR(velocities[0].linear.y, expected_vel.linear.y, 1e-5);
    EXPECT_NEAR(velocities[0].linear.z, expected_vel.linear.z, 1e-5);
}

// angular velocity
// TEST_F(TestConstantVestimator, test_angular_velocity_estimation) {
//     // Run the estimate function
//     constant_vestimator->estimate(velocities, transform_, veh_pose);

//     // Expected angular velocity
//     diviner::Velocity expected_ang_vel;
//     expected_ang_vel.angular.x = /* expected angular x */;
//     expected_ang_vel.angular.y = /* expected angular y */;
//     expected_ang_vel.angular.z = /* expected angular z */;

//     // Check estimated angular velocity
//     ASSERT_GE(velocities.size(), 1);
//     EXPECT_NEAR(velocities[0].angular.x, expected_ang_vel.angular.x, 1e-5);
//     EXPECT_NEAR(velocities[0].angular.y, expected_ang_vel.angular.y, 1e-5);
//     EXPECT_NEAR(velocities[0].angular.z, expected_ang_vel.angular.z, 1e-5);
// }


int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
