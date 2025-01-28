#ifndef UTILS__SWITCHER_HPP
#define UTILS__SWITCHER_HPP

#include <diviner/diviner.hpp>
#include <diviner/utils/types.hpp>

#include <ros/ros.h>

namespace diviner
{

struct SwitcherParams
{
    //
    bool debug=false;
};

class Switcher
{
    public:
        explicit Switcher(const SwitcherParams & params) : params_(params){};
        ~Switcher() = default;
        
        // lovely gps checker. if shits brok we hope this no brok
        void check_status();
    private:
        SwitcherParams params_;
};

}

#endif