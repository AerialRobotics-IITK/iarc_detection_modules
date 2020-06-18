cmake_minimum_required(VERSION 3.0.2)
project(plate_detector_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(plate_detector_ros_lib src/plate_detector_ros.cpp)

cs_add_executable(plate_detector_ros_node src/plate_detector_ros.cpp src/plate_detector_ros_node.cpp)
target_link_libraries(plate_detector_ros_node plate_detector_ros_lib) 

cs_install()
cs_export()