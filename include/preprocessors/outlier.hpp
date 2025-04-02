#ifndef PREPROCESSORS__OUTLIER_HPP
#define PREPROCESSORS__OUTLIER_HPP

#include <diviner/utils/types.hpp>
#include <preprocessors/i_preprocessor.hpp>

#include <pcl/filters/statistical_outlier_removal.h>

#include <vector>
#include <memory>
#include <list>

namespace preprocessor
{

struct OutlierProcessorParams
{
    bool debug = false;
};

class OutlierProcessor : public IPreprocessor
{
    public:
        explicit OutlierProcessor(const Params<OutlierProcessorParams, IPreprocessorParams> & params) : 
            IPreprocessor(params.parent_params),
            params_(params.child_params){};
        ~OutlierProcessor()=default;

        /**
         * Processing function to remove outliers from the input cloud
         *
         * @param input_cloud input cloud to process
         */
        void process(pcl::PointCloud<diviner::PointStamped>::Ptr input_cloud) override;
        
        std::string getName() override
        {
            return "Outlier Processor";
        }

    private:
        OutlierProcessorParams params_;
};

}

#endif