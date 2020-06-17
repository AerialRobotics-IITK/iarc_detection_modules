cmake_minimum_required(VERSION 3.0.2)
project(plate_detector)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(plate_detector_lib src/libplate_detection.cpp src/plate_detector.cpp)
target_link_libraries(plate_detector_lib ${OpenCV_LIBS})
cs_add_library(plate_pose_estimation_lib src/libpose_estimation.cpp src/pose_estimator.cpp)

cs_add_executable(plate_detector_node src/plate_detector_node.cpp)
target_link_libraries(plate_detector_node plate_detector_lib)

cs_add_executable(pose_estimator_node src/pose_estimator_node.cpp)
target_link_libraries(pose_estimator_node plate_pose_estimation_lib)

cs_install()
cs_export()
