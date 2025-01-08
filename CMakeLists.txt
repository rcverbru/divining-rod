cmake_minimum_required(VERSION 3.0.2)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros  
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  novatel_oem7_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  pcl_conversions
  pcl_ros  
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  novatel_oem7_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
##  Setup  ##
#############

set(DIVINER_SRCS
  src/diviner/diviner.cpp
)
add_library(diviner SHARED ${DIVINER_SRCS})

# interfaces
set(ALIGNER_SRCS
  src/aligner/example_aligner.cpp
  src/aligner/pcl_aligner.cpp
)

set(DESKEWER_SRCS
  src/deskewer/example_deskewer.cpp
  src/deskewer/standard_deskewer.cpp
)

set(FILTER_SRCS
  src/filter/example_filter.cpp
  src/filter/voxel_filter.cpp
)

set(MAP_SRCS
#   src/map/example_map.cpp
#   src/map/hash_map.cpp
#   src/map/kdtree_map.cpp
  src/map/octree_map.cpp
)

set(UTILS_SRCS
  src/utils/msg_converter.cpp
  src/utils/sync-er.cpp
)

set(VESTIMATOR_SRCS
  src/vestimator/example_vestimator.cpp
  src/vestimator/constant_vestimator.cpp
  src/vestimator/imu_vestimator.cpp
  src/vestimator/wheel_tick_vestimator.cpp
)

set(LOCALIZATION_SRCS
  ${ALIGNER_SRCS}
  ${DESKEWER_SRCS}
  ${FILTER_SRCS}
  ${MAP_SRCS}
  ${UTILS_SRCS}
  ${VESTIMATOR_SRCS}
)

add_library(localization SHARED ${LOCALIZATION_SRCS})
target_link_libraries(localization diviner)

#############
##  Build  ##
#############

# localization_node
add_executable(localization_node src/main.cpp src/localization_node.cpp)
target_link_libraries(localization_node localization diviner ${catkin_LIBRARIES})

#############
##  Tests  ##
#############

catkin_add_gtest(test_msg_converter test/utils/test_msg_converter.cpp)
if(TARGET test_msg_converter)
  target_link_libraries(test_msg_converter)
endif()

catkin_add_gtest(test_constant_vestimator test/vestimator/test_constant_vestimator.cpp)
if(TARGET test_constant_vestimator)
  target_link_libraries(test_constant_vestimator)
endif()