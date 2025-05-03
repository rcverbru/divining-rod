#include <gtest/gtest.h>

#include <map/voxel_map.hpp>

class TestVoxelMap : public ::testing::Test
{
    protected:
        diviner::Params<diviner::VoxelMapParams, diviner::IMapParams> voxel_map_params_;

        std::shared_ptr<diviner::VoxelMap> map_;

        pcl::PointCloud<diviner::PointStamped>::Ptr cloud =
        pcl::PointCloud<diviner::PointStamped>::Ptr(new pcl::PointCloud<diviner::PointStamped>);

        pcl::PointCloud<diviner::PointStamped>::Ptr output =
        pcl::PointCloud<diviner::PointStamped>::Ptr(new pcl::PointCloud<diviner::PointStamped>);

    void SetUp() override
    {
        voxel_map_params_.parent_params.debug = true;
        voxel_map_params_.child_params.debug = true;

        map_ = std::make_shared<diviner::VoxelMap>(voxel_map_params_);


    }

    void TearDown() override
    {
        if(map_ != nullptr)
        {
            map_->clear_map();
            map_.reset();
        }
    }
};

TEST_F(TestVoxelMap, test_map_initialize)
{
    // do it start?
}

TEST_F(TestVoxelMap, test_map_add_single_cloud)
{
    std::string scan_file = "/localization_ws/src/localization/test/map/monkey.pcd";

    if(pcl::io::loadPCDFile(scan_file, *cloud) < 0)
    {
        PCL_ERROR("Error loading cloud %s.\n", scan_file.c_str());
    }

    pcl::io::loadPCDFile(scan_file, *cloud);
    std::cout << "Loaded " << cloud->size() << " points into scan." << std::endl;

    map_->add_cloud(cloud);

    // need to compare the clouds to make sure they are the same
    pcl::io::savePCDFileASCII ("/localization_ws/single_add_map.pcd", *(map_->get_data()));
    EXPECT_TRUE(map_->size() > 0);
}

TEST_F(TestVoxelMap, test_map_add_multi_cloud)
{
    pcl::PointCloud<diviner::PointStamped>::Ptr expected_map =
    pcl::PointCloud<diviner::PointStamped>::Ptr(new pcl::PointCloud<diviner::PointStamped>);

    // Add multiple clouds and make sure the output map is what we're expecting
    std::string expected_map_file = "/localization_ws/src/localization/test/map/full.pcd";

    std::string first_scan_file = "/localization_ws/src/localization/test/map/top_half.pcd";
    std::string second_scan_file = "/localization_ws/src/localization/test/map/bottom_half.pcd";

    // Loading first half of monkey
    if(pcl::io::loadPCDFile(first_scan_file, *cloud) < 0)
    {
        PCL_ERROR("Error loading cloud %s.\n", first_scan_file.c_str());
    }

    pcl::io::loadPCDFile(first_scan_file, *cloud);
    std::cout << "Loaded " << cloud->size() << " points into scan." << std::endl;

    map_->add_cloud(cloud);

    // Loading second half of monkey
    if(pcl::io::loadPCDFile(second_scan_file, *cloud) < 0)
    {
        PCL_ERROR("Error loading cloud %s.\n", second_scan_file.c_str());
    }

    pcl::io::loadPCDFile(second_scan_file, *cloud);
    std::cout << "Loaded " << cloud->size() << " points into scan." << std::endl;

    map_->add_cloud(cloud);
    std::cout << "Added bottom half of monkey." << std::endl;

    // Loading expected_map for comparision
    if(pcl::io::loadPCDFile(expected_map_file, *expected_map) < 0)
    {
        PCL_ERROR("Error loading cloud %s.\n", expected_map_file.c_str());
    }

    pcl::io::loadPCDFile(expected_map_file, *expected_map);
    std::cout << "Loaded " << expected_map->size() << " points into expected_map." << std::endl;

    pcl::io::savePCDFileASCII ("/localization_ws/dual_add_map.pcd", *(map_->get_data()));

    //EXPECT_EQ(map_->get_data(), *expected_map);
    EXPECT_TRUE(map_->size() > 0);
}

TEST_F(TestVoxelMap, test_map_get_data)
{
    std::string scan_file = "/localization_ws/src/localization/test/map/monkey.pcd";
    
    if(pcl::io::loadPCDFile(scan_file, *cloud) < 0)
    {
        PCL_ERROR("Error loading cloud %s.\n", scan_file.c_str());
    }

    pcl::io::loadPCDFile(scan_file, *cloud);
    std::cout << "Loaded " << cloud->size() << " points into scan." << std::endl;

    pcl::PointCloud<diviner::PointStamped>::Ptr expected_map =
    pcl::PointCloud<diviner::PointStamped>::Ptr(new pcl::PointCloud<diviner::PointStamped>);

    *expected_map = *cloud;

    map_->add_cloud(cloud);

    output = map_->get_data();
    //EXPECT_EQ(*output, *expected_map);
    EXPECT_TRUE(output->size() == expected_map->size());
}

// Apply transform function not used in code
// Should probably remove this down the line from all map interfaces
// TEST_F(TestVoxelMap, test_map_apply_transform)

TEST_F(TestVoxelMap, test_map_clear_map)
{
    map_->clear_map();

    EXPECT_TRUE(map_->size() == 0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}
