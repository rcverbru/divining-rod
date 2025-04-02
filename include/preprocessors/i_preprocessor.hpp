#ifndef PREPROCESSORS__I_PREPROCESSOR_HPP
#define PREPROCESSORS__I_PREPROCESSOR_HPP

#include <diviner/utils/types.hpp>

#include <vector>
#include <memory>
#include <list>

namespace preprocessor
{

template<typename CHILD_PARAMS, typename PARENT_PARAMS>
struct Params
{
    CHILD_PARAMS child_params;
    PARENT_PARAMS parent_params;
};

struct IPreprocessorParams
{
    bool debug = false;
};

class IPreprocessor
{
    public:
        explicit IPreprocessor(const IPreprocessorParams & params) :
        params_(params){};
        ~IPreprocessor()=default;

        /**
         * main process function for the preprocessor
         * @param input_cloud The input cloud to be processed
         */
        virtual void process(pcl::PointCloud<diviner::PointStamped>::Ptr input_cloud) = 0;

        /**
         * getName function to get the name of the preprocessor
         * @return The name of the preprocessor
         */
        virtual std::string getName() = 0;

    private:
        IPreprocessorParams params_;
};

}

#endif