cmake_minimum_required(VERSION 3.0.2)
project(led_detector)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(led_detector_lib src/libled_detection.cpp src/led_detector.cpp)
target_link_libraries(led_detector_lib ${OpenCV_LIBS})

cs_add_executable(led_detector_node src/led_detector_node.cpp)
target_link_libraries(led_detector_node led_detector_lib)

cs_install()
cs_export()