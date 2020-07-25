cmake_minimum_required(VERSION 2.8.3)
project(kfast_iarc)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Ofast -std=gnu++17 -fomit-frame-pointer -mavx2 -march=native -mfma -flto -funroll-all-loops -fpeel-loops -ftracer -ftree-vectorize")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Ofast -std=c11 -fomit-frame-pointer -mavx2 -march=native -mfma -flto -funroll-all-loops -fpeel-loops -ftracer -ftree-vectorize")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

cs_add_library(kfast_iarc_lib src/kfast_iarc.cpp)

cs_add_executable(kfast_iarc_node src/kfast_iarc_node.cpp)

target_link_libraries(kfast_iarc_node kfast_iarc_lib)

target_link_libraries(kfast_iarc_node ${OpenCV_LIBRARIES})

cs_install()
cs_export()