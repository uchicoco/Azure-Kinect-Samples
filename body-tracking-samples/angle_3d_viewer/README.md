# Azure Kinect Body Tracking Simple3dViewer Sample

## Introduction

The Azure Kinect Body Tracking Simple3dViewer sample creates a 3d window that visualizes all the information provided
by the body tracking SDK.

## Usage Info

USAGE: simple_3d_viewer.exe SensorMode[NFOV_UNBINNED, WFOV_BINNED](optional) RuntimeMode[CPU, OFFLINE](optional)
* SensorMode:
  * NFOV_UNBINNED (default) - Narraw Field of View Unbinned Mode [Resolution: 640x576; FOI: 75 degree x 65 degree]
  * WFOV_BINNED             - Wide Field of View Binned Mode [Resolution: 512x512; FOI: 120 degree x 120 degree]
* RuntimeMode:
  * CPU - Use the CPU only mode. It runs on machines without a GPU but it will be much slower
  * OFFLINE - Play a specified file. Does not require Kinect device. Can use with CPU mode

e.g.   simple_3d_viewer.exe WFOV_BINNED CPU
                 simple_3d_viewer.exe CPU
                 simple_3d_viewer.exe WFOV_BINNED
                 simple_3d_viewer.exe OFFLINE MyFile.mkv

## Instruction

### Basic Navigation:
* Rotate: Rotate the camera by moving the mouse while holding mouse left button
* Pan: Translate the scene by holding Ctrl key and drag the scene with mouse left button
* Zoom in/out: Move closer/farther away from the scene center by scrolling the mouse scroll wheel
* Select Center: Center the scene based on a detected joint by right clicking the joint with mouse

### Key Shortcuts
* ESC: quit
* h: help
* b: body visualization mode
* k: 3d window layout

# Azure Kinect Body Tracking 3D Viewer - Ubuntu 18.04 Setup

This README provides instructions for building and running the Azure Kinect Body Tracking 3D Viewer on Ubuntu 18.04.

## Prerequisites

1. Install the Azure Kinect SDK and Body Tracking SDK:

2. Install build dependencies:

3. If using visualization with OpenGL:

## Building the Project

1. Make the build script executable:

2. Run the build script:

3. The executable will be created in the `build` directory.

## Running the Application

Run the application from the build directory:

## Common Issues and Fixes

### Filesystem Library Issues

Ubuntu 18.04 has the filesystem library as an experimental feature. If you encounter issues with `std::filesystem`, make sure you're compiling with C++17 and linking against `stdc++fs`.

### Access to Azure Kinect Camera

To access the Azure Kinect device without sudo privileges, create a udev rule:

### CUDA/TensorRT Support

To use CUDA or TensorRT processing modes, make sure you have installed:
1. NVIDIA drivers
2. CUDA Toolkit
3. TensorRT (if needed)

Then update your LD_LIBRARY_PATH as needed:

### Note on DirectML Processing Mode

DirectML processing mode is only available on Windows, as indicated by the conditional code in `main.cpp`. On Ubuntu, you should use either CPU, CUDA, or TensorRT processing modes.
