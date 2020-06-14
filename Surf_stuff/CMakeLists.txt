cmake_minimum_required(VERSION 2.8)
project(CV)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Surf1 Surf1.cpp)
target_link_libraries(Surf1 ${OpenCV_LIBS})