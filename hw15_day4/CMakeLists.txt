# CMake 最低版本号要求2.8
cmake_minimum_required(VERSION 3.16)

# 项目名称feature_match
project(feature_match)

# 设置编译方式Release
set(CMAKE_BUILD_TYPE "Release")

# 支持C++11 -o3优化
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# 查找系统里的OpenCV，找不到输出警告信息
find_package(OpenCV 3.0 REQUIRED)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

# 添加项目包含路径
include_directories(${PROJECT_SOURCE_DIR}/include
        ${OpenCV_DIRS})

add_library(vfc SHARED ${PROJECT_SOURCE_DIR}/source/vfc.cpp)


# 用源文件为工程添加可执行文件，名字和项目名称一致
add_executable(feature_match source/feature_matcher.cpp)

# 可执行文件添加库链接
target_link_libraries(feature_match
        vfc
        ${OpenCV_LIBS})

