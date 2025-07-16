
# Azure Kinect Angle 3D Viewer

This project is a sample application for Azure Kinect Body Tracking, providing 3D visualization and CSV export of joint positions and calculated angles. It supports both live device streaming and offline playback of recorded `.mkv` files. The application is designed for Linux and Windows, with CUDA, CPU, and TensorRT processing modes.

## Features
- Real-time 3D visualization of tracked bodies and joints
- Export joint positions and calculated angles (right arm, left arm, legs) to CSV
- Save color images at specified intervals
- Named pipe output for angle data (Linux)
- Customizable coordinate transformation for joint positions
- Multiple runtime modes: CPU, CUDA, TensorRT, DirectML (Windows)
- Offline playback from `.mkv` files

## Requirements
- Azure Kinect SDK and Body Tracking SDK
- Eigen library (included in `additional_includes`)
- CMake (build system)
- Linux or Windows (tested on Ubuntu)
- CUDA-capable GPU for CUDA/TensorRT modes

## Build Instructions

### Linux
```bash
# From the project root
git submodule update --init --recursive
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### Windows
Use Visual Studio to open `angle_3d_viewer.vcxproj` or use CMake as above.

## Usage

```bash
./angle_3d_viewer [SensorMode] [RuntimeMode] [options]
```

### Sensor Modes
- `NFOV_UNBINNED` (default): Narrow Field of View
- `WFOV_BINNED`: Wide Field of View

### Runtime Modes
- `CPU`: CPU only
- `CUDA`: GPU CUDA
- `TENSORRT`: GPU TensorRT
- `DIRECTML`: GPU DirectML (Windows only)
- `OFFLINE`: Play a recorded file (requires `.mkv` filename)

### Options
- `-model MODEL_PATH`: Specify custom model path
- `-csv filename.csv`: Output CSV file name (default: `joint_positions.csv`)
- `-novis`: Disable visualization (CSV only)
- `-img FREQ`: Save color images every FREQ frames (default folder: `color_images`)
- `FPS_5`, `FPS_15`, `FPS_30`: Set camera FPS

### Examples
```bash
# Live device, default settings
./angle_3d_viewer

# Live device, wide FOV, CUDA
./angle_3d_viewer WFOV_BINNED CUDA

# Offline playback, no visualization
./angle_3d_viewer OFFLINE MyFile.mkv -novis

# Save color images every 10 frames
./angle_3d_viewer CUDA -img 10
```

## Output
- **CSV File**: Contains body ID, timestamp, joint positions (with coordinate transformation), confidence, and calculated angles.
- **Named Pipe**: On Linux, angle data is sent to `/tmp/angle_data_pipe` for inter-process communication.
- **Color Images**: Saved as JPEG in the specified folder.

## Navigation & Controls
- **Rotate**: Mouse left button drag
- **Pan**: Ctrl + Mouse left button drag
- **Zoom**: Mouse scroll wheel
- **Select Center**: Right-click joint
- **ESC**: Quit
- **h**: Help
- **b**: Toggle body visualization
- **k**: Change 3D window layout

## Code Structure
- `main.cpp`: Main application logic, argument parsing, visualization, CSV export
- `Addition.cpp/h`: Coordinate transformation, CSV writing, image saving
- `AngleCalculator.cpp/h`: Angle calculation using projected joint positions
- `Pipe.cpp/h`: Named pipe management and angle data output
- `additional_includes/`: Eigen library and other dependencies

## License
This project is licensed under the MIT License. See `LICENSE` and individual COPYING files for third-party dependencies.

## Acknowledgements
- Microsoft Azure Kinect SDK
- Eigen library

## Troubleshooting
- Ensure all dependencies are installed and environment variables are set for Azure Kinect SDK
- For CUDA/TensorRT modes, verify GPU compatibility and driver installation
- On Linux, ensure permissions for `/tmp/angle_data_pipe`

## Contact
For issues or contributions, please open an issue or pull request on the repository.

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
