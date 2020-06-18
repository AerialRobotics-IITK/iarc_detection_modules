cmake_minimum_required(VERSION 3.0.2)
project(plate_detector)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(plate_detector_lib src/plate_detector.cpp)
target_link_libraries(plate_detector_lib ${OpenCV_LIBS})

cs_install()
cs_export()