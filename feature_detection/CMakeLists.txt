cmake_minimum_required(VERSION 2.8.3)
project(feature_detection)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

cs_add_library(text_detect_lib src/text_detect.cpp)

cs_add_executable(text_detect_node src/text_detect_node.cpp)

target_link_libraries(text_detect_node text_detect_lib)

target_link_libraries(text_detect_node ${OpenCV_LIBRARIES})

cs_install()
cs_export()