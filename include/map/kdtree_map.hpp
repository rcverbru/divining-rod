#ifndef MAP__KDTREE_MAP_HPP
#define MAP__KDTREE_MAP_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_map.hpp>

#include <pcl/kdtree/kdtree_flann.h>

namespace diviner
{

struct KdTreeMapParams
{
    double max_distance;
    bool debug = false;
};

class KdTreeMap : public IMap
{
    public:
        explicit KdTreeMap(const Params<KdTreeMapParams, IMapParams> & params) : 
        IMap(params.parent_params),
        params_(params.child_params){};
        ~KdTreeMap() = default;

        void add_point() {};

        void add_cloud(const pcl::PointCloud<diviner::PointStamped>::Ptr input_cloud, std::shared_ptr<diviner::IMap> map_) override {};

        void clear_map() override {};
        
        int capacity() override {};

        size_t size() override {};

        /**
         * Trims down the current local map by removing points 
         * outside of the max distance range
         * 
         * @param 
         */
        void trim_map() override;

    private:
        KdTreeMapParams params_;
        pcl::KdTreeFLANN<diviner::PointStamped> map_;

};

}

#endif