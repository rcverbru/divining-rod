#ifndef MAP__HASH_MAP_HPP
#define MAP__HASH_MAP_HPP
// 47.11056830682432, -88.58620657853993

#include <diviner/i_map.hpp>

#include <unordered_map>
#include <vector>

namespace diviner
{

struct HashMapParams
{
    double voxel_size;
    double max_distance;
    int max_points_per_voxel;
    bool debug = false;
};

// class HashMap : public IMapperMap
// {
//     public:
//         explicit HashMap(const Params<HashMapperParams, IMapperParams> & params, size_t num_vertices, bool directed = false) : 
//         IMapperMap(params.parent_params), 
//         params_(params.child_params), 
//         NUM_VERTICES_(num_vertices), 
//         DIRECTED_(directed)
//         //explicit HashMap(size_t num_vertices, bool directed = false) : NUM_VERTICES_(num_vertices), DIRECTED_(directed)
//         {
//             map_ = new NodeType[NUM_VERTICES_]
//         };
        
//         ~HashMap()
//         {
//             if(map_ !=nullptr)
//             {
//                 delete[] map_;
//             }
//         };

//         void clear_map() 
//         {
//             map_.clear();
//         };
        
//         int capacity()
//         {
//             return map_.capacity();
//         };

//         //std::vector<Eigen::Vector3d> map_;

//     private:
//         HashMapperParams params_;
        
// };
class HashMap : public IMap
{
    public:
        explicit HashMap(const Params<HashMapParams, IMapParams> & params) : 
        IMap(params.parent_params),
        params_(params.child_params){};
        ~HashMap() = default;

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

        std::vector<Eigen::Vector3d> map_;

    private:
        HashMapParams params_;
        
};

}

#endif