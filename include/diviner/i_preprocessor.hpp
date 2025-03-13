#ifndef DIVINER__I_PREPROCESSOR_HPP
#define DIVINER__I_PREPROCESSOR_HPP

#include <diviner/utils/types.hpp>

#include <vector>
#include <memory>
#include <list>

namespace diviner
{

struct IPreprocessorParams
{
    bool debug = false;
};

class IPreprocessor
{
    public:
        explicit IPreprocessor(const IPreprocessorParams & params) : params_(params){};
        ~IPreprocessor()=default;

        /**
         * main process function for the preprocessor
         * @param input_cloud The input cloud to be processed
         */
        virtual void process(pcl::PointCloud<diviner::PointStamped>::Ptr input_cloud) = 0;

    private:
        IPreprocessorParams params_;
};

}

#endif