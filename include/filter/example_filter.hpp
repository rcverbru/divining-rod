#ifndef FILTER__EXAMPLE_FILTER_HPP
#define FILTER__EXAMPLE_FILTER_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_filter.hpp>

namespace diviner
{

struct ExampleFilterParams
{
    bool debug = false;
};

class ExampleFilter : public IFilter
{
    public:
        explicit ExampleFilter(const Params<ExampleFilterParams, IFilterParams> & params) :
        IFilter(params.parent_params),
        params_(params.child_params){};
        ~ExampleFilter() = default;
        
        void filter(pcl::PointCloud<diviner::PointStamped>::Ptr point_cloud) override;

    private:
        ExampleFilterParams params_;
};

}

#endif