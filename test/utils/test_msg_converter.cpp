#include <gtest/gtest.h>
#include <diviner/diviner.hpp>
#include <utils/msg_converter.hpp>

#include <memory>

class TestMsgConverter : public ::testing::Test {
    protected:
        MsgConverter converter;
    void SetUp() override {

    }
};

// Need to update this with new stuff... Commenting it out because it'll cause more errors :(
TEST_F(TestMsgConverter, from_ros){
    sensor_msgs::PointCloud2ConstPtr input_cloud;
    input_cloud->data = [10, 215, 143, 64, 10, 215, 41, 65];
    diviner::CeptonPoint result;

    converter.from_ros(input_cloud, diviner::CeptonPoint result);
    
    diviner::CeptonPoint expected;
    
    expected.x = 10;
    expected.y = 215;
    expected.z = 143;
    expected.reflectivity = 64;
    expected.relative_timestamp = 10;
    expected.flags = 215;
    expected.channel_id = 41;
    expected.valid = 65;
    
    EXPECT_EQ(result, expected);
}