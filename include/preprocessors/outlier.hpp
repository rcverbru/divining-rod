#ifndef PREPROCESSORS__OUTLIER_HPP
#define PREPROCESSORS__OUTLIER_HPP

#include <diviner/utils/types.hpp>
#include <diviner/i_preprocessor.hpp>

#include <pcl/filters/statistical_outlier_removal.h>

#include <vector>
#include <memory>
#include <list>

namespace diviner
{

struct OutlierProcessorParams
{
    bool debug = false;
};

class OutlierProcessor
{
    public:
        explicit OutlierProcessor(const OutlierProcessorParams & params) : params_(params){};
        ~OutlierProcessor()=default;

        /**
         * Processing function to remove outliers from the input cloud
         *
         * @param input_cloud input cloud to process
         */
        void process(pcl::PointCloud<diviner::PointStamped>::Ptr input_cloud) override;
        

    private:
        OutlierProcessorParams params_;
};

}

#endif