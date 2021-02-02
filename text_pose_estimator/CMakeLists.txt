cmake_minimum_required(VERSION 3.0.2)
project(text_pose_estimator)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(${PROJECT_NAME}_lib
    src/text_libpose_estimation.cpp
    src/text_pose_estimator.cpp
)

cs_add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

cs_install()
cs_export()
