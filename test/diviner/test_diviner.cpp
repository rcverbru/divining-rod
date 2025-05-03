#include <gtest/gtest.h>

#include <diviner/diviner.hpp>

class TestDiviner : public ::testing::Test {
    protected:
        std::shared_ptr<diviner::Diviner> diviner_;
        
        diviner::PclAlignerParams pcl_aligner_params_;
        std::shared_ptr<diviner::PclAligner> aligner_;

        diviner::ExampleDeskewerParams example_deskewer_params_;
        std::shared_ptr<diviner::ExampleDeskewer> deskewer_;

        diviner::VoxelFilterParams voxel_filter_params_;
        std::shared_ptr<diviner::VoxelFilter> filter_;

        diviner::VoxelMapParams voxel_map_params_;
        std::shared_ptr<diviner::VoxelMap> map_;

        diviner::ConstantVestimatorParams constant_vestimator_params_;
        std::shared_ptr<diviner::ConstantVestimator> vestimator_;

    void SetUp() override {
        aligner_ = std::make_shared<diviner::PclAligner>(pcl_aligner_params_);
        deskewer_ = std::make_shared<diviner::ExampleDeskewer>(example_deskewer_params_);
        filter_ = std::make_shared<diviner::VoxelFilter>(voxel_filter_params_);
        map_ = std::make_shared<diviner::VoxelMap>(voxel_map_params_);
        vestimator_ = std::make_shared<diviner::ConstantVestimator>(constant_vestimator_params_);

    }
};

TEST_F(TestDiviner, test_diviner_step) {

}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
