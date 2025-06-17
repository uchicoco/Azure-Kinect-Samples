#!/bin/bash

# Build script for simple_3d_viewer on Ubuntu 18.04

# Exit on error
set -e

# Create build directory
mkdir -p build
cd build

# Configure with CMake
cmake ..

# Build
make -j $(nproc)

echo "Build completed successfully!"
echo "Run with: ./simple_3d_viewer"
