# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.10)
project(simple_3d_viewer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(../sample_helper_libs/window_controller_3d window_controller_3d)

add_executable(simple_3d_viewer
       main.cpp
       Addition.cpp
       AngleCalculator.cpp)

target_include_directories(simple_3d_viewer PRIVATE ../sample_helper_includes)

#find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

# Dependencies of this library
target_link_libraries(simple_3d_viewer PRIVATE 
    k4a
    k4abt
    k4arecord
    window_controller_3d::window_controller_3d
    glfw
    dl
    )
