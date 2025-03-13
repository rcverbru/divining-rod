#ifndef MAP__OCTREE_MAP_HPP
#define MAP__OCTREE_MAP_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_map.hpp>

// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/point_cloud_handlers.h>
// #include <pcl/visualization/common/common.h>

// #include <pcl/octree/octree.h>
// #include <pcl/octree/octree_impl.h>

#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>

namespace diviner
{

struct OctreeMapParams
{
    double octree_depth = 10; // Not necessary
    double octree_resolution = 1;
    double max_distance = 40;
    bool output_map = false;
    bool debug = false;
};

class OctreeMap : public IMap
{
    public:
        explicit OctreeMap(const Params<OctreeMapParams, IMapParams> & params) : 
        IMap(params.parent_params),
        params_(params.child_params){

            local_map_octree = boost::make_shared<pcl::octree::OctreePointCloud<diviner::PointStamped>>(params_.octree_resolution);
            local_map_pointcloud = boost::make_shared<pcl::PointCloud<diviner::PointStamped>>();
            // indices_list = boost::make_shared<pcl::IndicesConstPtr>();

            // LeafContainer = 
            // BranchContainer = 
            // local_map_octree->setTreeDepth(params_.octree_depth);
            local_map_octree->setResolution(params_.octree_resolution);
            local_map_pointcloud->header.frame_id = "cepton2";

        };
        ~OctreeMap()
        {
            if(local_map_octree != nullptr)
            {
                clear_map();
            }
        }

        void add_cloud(const pcl::PointCloud<diviner::PointStamped>::Ptr input_cloud) override;

        pcl::PointCloud<diviner::PointStamped>::Ptr get_data() override;

        void get_voxels();

        void clear_map() override;
        
        int capacity() override;
        
        size_t size() override;

        /**
         * Trims down the current local map by removing points 
         * outside of the max distance range
         * 
         * @param 
         */
        void trim_map() override;

    private:
        OctreeMapParams params_;
        
        // pcl::IndicesPtr test_indices = boost::make_shared<std::vector<int>>();

        pcl::octree::OctreePointCloud<diviner::PointStamped>::Ptr local_map_octree;
        pcl::PointCloud<diviner::PointStamped>::Ptr local_map_pointcloud;
        // pcl::IndicesConstPtr indices_list;
        std::vector<diviner::PointStamped, Eigen::aligned_allocator<diviner::PointStamped>> voxel_centers;

        pcl::IndicesConstPtr indices_test = boost::make_shared<std::vector<int>>();

};

}

#endif