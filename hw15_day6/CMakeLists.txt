cmake_minimum_required(VERSION 3.25)
project(transform)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/include/eigen3)

add_executable(transform transform.cpp)