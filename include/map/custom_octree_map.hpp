#ifndef MAP__OCTREE_MAP_HPP
#define MAP__OCTREE_MAP_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_map.hpp>

namespace diviner
{

struct OctreeMapParams
{
    float max_size = 500; // in meters
    float min_voxel_size = 1; // in meters
    float max_points_voxel = 25;
    Eigen::Vector3d origin;
    Eigen::Vector3d halfDimension;

    bool debug = false;
};

class OctreeMap : public IMap
{
    public:
        explicit OctreeMap(const Params<OctreeMapParams, IMapParams> & params) : 
        IMap(params.parent_params),
        params_(params.child_params)
        {
            for(int i = 0; i < 8; i++)
            {
                children[i] = NULL;
            }

        };

        ~OctreeMap()
        {
            for(int i = 0; i < 8; i++)
            {
                delete children[i];
        }

        // Make custom octree
        Eigen::Vector3d origin;
        Eigen::Vector3d halfDimension;

        Octree *children[8];
        OctreePoint *data;

        void add_cloud(const pcl::PointCloud<diviner::PointStamped>::Ptr input_cloud) override;

        pcl::PointCloud<diviner::PointStamped>::Ptr get_data() override;

        void getLowestVoxels();

        void getOctreeDepth();

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
        
};

}

#endif
