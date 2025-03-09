#ifndef MAP__EXAMPLE_MAP_HPP
#define MAP__EXAMPLE_MAP_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_map.hpp>

namespace diviner
{

struct ExampleMapParams
{
    bool debug = false;
};

class ExampleMap : public IMap
{
    public:
        explicit ExampleMap(const Params<ExampleMapParams, IMapParams> & params) : 
        IMap(params.parent_params),
        params_(params.child_params){};
        ~ExampleMap() = default;

        void add_cloud(const pcl::PointCloud<diviner::PointStamped>::Ptr input_cloud) override {};

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

        std::vector<Eigen::Vector3d> map_;

    private:
        ExampleMapParams params_;
        
};

}

#endif