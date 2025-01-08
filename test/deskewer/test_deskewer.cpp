#include <gtest/gtest.h>

#include <src/localization/include/deskewer/standard_deskewer.hpp>>

class TestDeskewer : public ::testing::Test {
    protected:
        //Not sure what goes here yet
    void SetUp() override {

    }
};

TEST_F(TestDeskewer, what_does_it_do) {

}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}