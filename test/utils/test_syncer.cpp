#include <gtest/gtest.h>
#include <utils/syncer.hpp>
#include <diviner/diviner.hpp>
#include <diviner/utils/types.hpp>

#include <queue>


class TestSyncer : public ::testing::Test {
    protected:
        diviner::SyncerParams syncer_params_;

        diviner::Syncer t_syncer = diviner::Syncer(syncer_params_);
        
        std::queue<geometry_msgs::PoseStamped> gps_queue;
        geometry_msgs::PoseStamped gps;

        std::queue<pcl::PointCloud<diviner::PointStamped>> cloud_queue; 
        pcl::PointCloud<diviner::PointStamped> cloud;

        ros::Time gps_time;

        diviner::SyncedMsgs synced_out, expected;


    void SetUp() override {

        uint64_t gps_vals[4] = {1731776843781336007, 1731776843881336007, 1731776843921336007, 1731776843992556000};
        uint64_t cloud_vals = 1731776843923136000;                   
        
        cloud.push_back(1);
        cloud.header.stamp = cloud_vals;
        cloud_queue.push(cloud);
          
        for (u_int8_t j = 0; j < 4; j++)
        {
            gps_time.fromNSec(gps_vals[j]);
            gps.header.stamp = gps_time;
            gps_queue.push(gps);
        }    
        
    }
};

TEST_F(TestSyncer, test_sync_timestamps) {

    synced_out = t_syncer.sync(cloud_queue, gps_queue);

    expected.gps.header.stamp = gps_time.fromNSec(1731776843921336007); 
    
    // std::cout << "  - syncer: Expected GPS timestamp = " << expected.gps.header.stamp << std::endl;
    // std::cout << "  - syncer: Output GPS timestamp = " << synced_out.gps.header.stamp << std::endl;

    EXPECT_EQ(synced_out.gps.header.stamp, expected.gps.header.stamp);
    
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}