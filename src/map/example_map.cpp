#include <map/example_map.hpp>

namespace diviner
{

void ExampleMap::trim_map()
{
    // remove points over a certain distance away from the ego vehicle.
    if(params_.debug)
    {
        std::cout << "Deleting the map" << std::endl;        
    }

}

}