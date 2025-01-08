#ifndef FILTER__VOXEL_FILTER_HPP
#define FILTER__VOXEL_FILTER_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_filter.hpp>

#include <pcl/filters/voxel_grid.h>

#include <vector>
#include <list>
#include <memory>

namespace diviner
{

struct VoxelFilterParams
{
    std::vector<float> leaf_size;
    bool debug = true;
};

class VoxelFilter : public IFilter
{
    public:
        explicit VoxelFilter(const Params<VoxelFilterParams, IFilterParams> & params) :
        IFilter(params.parent_params),
        params_(params.child_params){};
        ~VoxelFilter() = default;
        
        void filter(pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud) override;

    private:
        VoxelFilterParams params_;
};

}

#endif