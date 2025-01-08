#include <gtest/gtest.h>

#include <src/localization/include/filter/voxel_filter.hpp>

class TestFilter : public ::testing::Test {
    protected:
        //Not sure what goes here yet
    void SetUp() override {

    }
};

TEST_F(TestFilter, what_does_it_do) {

}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}