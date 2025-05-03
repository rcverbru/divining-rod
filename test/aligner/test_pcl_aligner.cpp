#include <gtest/gtest.h>

#include <aligner/pcl_aligner.hpp>
#include <map/voxel_map.hpp>
#include <diviner/utils/types.hpp>

#include <iostream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>


class TestAligner : public ::testing::Test
{
    protected:
        diviner::Params<diviner::PclAlignerParams, diviner::IAlignerParams> pcl_aligner_params_;
        diviner::Params<diviner::VoxelMapParams, diviner::IMapParams> voxel_map_params_;

        std::shared_ptr<diviner::PclAligner> aligner_;
        std::shared_ptr<diviner::VoxelMap> map_;

        pcl::PointCloud<diviner::PointStamped>::Ptr cloud_in = 
        pcl::PointCloud<diviner::PointStamped>::Ptr(new pcl::PointCloud<diviner::PointStamped>);
        pcl::PointCloud<diviner::PointStamped>::Ptr scan = 
        pcl::PointCloud<diviner::PointStamped>::Ptr(new pcl::PointCloud<diviner::PointStamped>);

        double fitness_score, expected_score;

    void SetUp() override
    {
        // Cloud File
        std::string map_file = "/home/rcv/dev/rse/localization_ws/src/localization/test/map.pcd";
        std::string scan_file = "/home/rcv/dev/rse/localization_ws/src/localization/test/scan.pcd";

        // Set Params
        pcl_aligner_params_.parent_params.debug = true;
        pcl_aligner_params_.child_params.debug = true;
        pcl_aligner_params_.child_params.convergence_criterion = 0.01;
        pcl_aligner_params_.child_params.num_iterations = 30;
        pcl_aligner_params_.child_params.alignment_state = "set";
        voxel_map_params_.parent_params.debug = true;

        // Make the pointers
        aligner_ = std::make_shared<diviner::PclAligner>(pcl_aligner_params_);
        map_ = std::make_shared<diviner::VoxelMap>(voxel_map_params_);

        if (pcl::io::loadPCDFile(map_file, *cloud_in) < 0)
        {
            PCL_ERROR ("Error loading cloud %s.\n", map_file.c_str());
        }

        pcl::io::loadPCDFile(map_file, *cloud_in);
        std::cout << "Loaded " << cloud_in->size() << " points into map." << std::endl;
        map_->add_cloud(cloud_in);

        if (pcl::io::loadPCDFile(scan_file, *cloud_in) < 0)
        {
            PCL_ERROR ("Error loading cloud %s.\n", scan_file.c_str());
        }

        pcl::io::loadPCDFile(scan_file, *scan);
        std::cout << "Loaded " << scan->size() << " points into scan." << std::endl;
    }
 
    void TearDown() override
    {
        if(aligner_ != nullptr)
        {
            aligner_.reset();
        }
        if(map_ != nullptr)
        {
            map_->clear_map();
            map_.reset();
        }
    }
};

TEST_F(TestAligner, test_align_initialize)
{
    // Do it start?
}

TEST_F(TestAligner, test_align_convergence)
{
    const auto &[alignment, stats] = aligner_->align(scan, map_);

    EXPECT_EQ(alignment, alignment);
}

TEST_F(TestAligner, test_align_alignment)
{
    const auto &[alignment, stats] = aligner_->align(scan, map_);

    //EXPECT_TRUE();
}

TEST_F(TestAligner, test_align_threshold)
{
    const auto &[alignment, stats] = aligner_->align(scan, map_);
    
    EXPECT_TRUE(stats.correspondence < stats.target_threshold);
}

TEST_F(TestAligner, test_align_update_points)
{
    //
}

TEST_F(TestAligner, test_align_pose_predict)
{
    //
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}
