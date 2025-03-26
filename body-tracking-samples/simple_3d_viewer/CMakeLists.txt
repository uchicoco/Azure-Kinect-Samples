# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.10)
project(simple_3d_viewer)

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Find Azure Kinect SDK packages
find_package(k4a REQUIRED)
find_package(k4abt REQUIRED)
find_package(k4arecord REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/additional_includes
)

# Source files
set(SOURCES
    main.cpp
    Addition.cpp
    AngleCalculator.cpp
)

# Header files
set(HEADERS
    Addition.h
    AngleCalculator.h
)

# Create the executable
add_executable(simple_3d_viewer ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(simple_3d_viewer
    k4a::k4a
    k4a::k4abt
    k4a::k4arecord
    stdc++fs  # For std::filesystem support on Ubuntu 18.04
    pthread
    dl
)

# If OpenGL visualization is needed, uncomment these lines
# find_package(OpenGL REQUIRED)
# find_package(GLFW3 REQUIRED)
# target_link_libraries(simple_3d_viewer OpenGL::GL glfw)

