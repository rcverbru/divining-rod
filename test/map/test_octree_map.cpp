#include <gtest/gtest.h>
#include <map/octree_map.hpp>

class TestOctreeMap : public ::testing::Test {
protected:
    diviner::Params<diviner::OctreeMapParams, diviner::IMapParams> params;
    std::shared_ptr<diviner::ConstantVestimator> octree_map;
    std::vector<geometry_msgs::PoseStamped> veh_pose;

    void SetUp() override {

        // Create the ConstantVestimator with the correct parameters type
        octree_map = std::make_shared<diviner::OctreeMap>(params);
    }

    void TearDown() override {
        if (octree_map != nullptr) {
            octree_map.reset();
        }
    }
};

TEST_F(TestOctreeMap, initialization_without_segfault) {
    // Test that the estimator initializes without errors
}

TEST_F(TestConstantVestimator, test_removal_max_dist_points) {
    // Run the estimate function
    octree_map->trim_map();

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


int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
