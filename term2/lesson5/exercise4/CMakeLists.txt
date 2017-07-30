project(exercise4)

cmake_minimum_required(VERSION 3.5)


add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")



#include_directories(Eigen)
#add_subdirectory(Eigen)

add_executable(main main.cpp)
