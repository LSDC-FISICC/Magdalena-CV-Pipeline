# Magdalena CV Pipeline

## Overview

The Magdalena Pipeline is a ROS 2-based system for **NDVI (Normalized Difference Vegetation Index) analysis and monitoring** using Intel RealSense cameras. This pipeline captures multispectral imagery from RealSense depth and RGB cameras to compute vegetation indices for agricultural and environmental monitoring applications.

### Key Components

- **ndvi_surco1**: Main ROS 2 package for NDVI processing and analysis
- **realsense-ros**: ROS 2 wrapper for Intel RealSense cameras (depth and RGB sensors)
- **vision_opencv**: Integration layer between OpenCV and ROS 2 for image processing

### Use Cases

- Real-time vegetation health monitoring
- Precision agriculture analysis
- Environmental crop assessment
- Plant growth tracking and analysis

---

## Building the Workspace

### Prerequisites

Ensure you have the following installed:
- **ROS 2** (verify with `ros2 --version`)
- **colcon** build tool:
  ```bash
  sudo apt install python3-colcon-common-extensions
  ```

### Build Steps

1. **Navigate to your workspace root:**
   ```bash
   cd ~/ros2_ws
   ```

2. **Source the ROS 2 environment** (if not already done):
   ```bash
   source /opt/ros/<distro>/setup.bash
   ```
   Replace `<distro>` with your ROS 2 distribution (e.g., `humble`, `iron`, `jazzy`)

3. **Install system dependencies** (first time only):
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the entire workspace:**
   ```bash
   colcon build
   ```

5. **Source the build environment:**
   ```bash
   source install/setup.bash
   ```

### Build Specific Packages

If you only want to build certain packages:
```bash
# Build only the NDVI package
colcon build --packages-select ndvi_surco1

# Build only the RealSense wrapper
colcon build --packages-select realsense2_camera

# Build OpenCV components
colcon build --packages-select cv_bridge
```

### Verification

After successful build, you should see the following directories in your workspace root:
- `build/` - Intermediate build files
- `install/` - Installation directory with executables and libraries
- `log/` - Build logs

---

## Running the Pipeline

After building and sourcing the setup script:

```bash
# Launch the RealSense camera node and NDVI processing
ros2 launch ndvi_surco1 rs_launch.py
```

For additional launch options, refer to the individual package documentation in their respective directories.

---

## Project Structure

```
Magdalena---Pipeline/
├── ndvi_surco1/          # Main NDVI processing package
├── realsense-ros/        # Intel RealSense camera drivers
├── vision_opencv/        # OpenCV-ROS integration
└── README.md             # This file
```

---

## Requirements

- Intel RealSense camera (D435, D455, or compatible)
- Linux system (Ubuntu 20.04+)
- ROS 2 (Foxy, Humble, Iron, or Jazzy)
- Python 3.8+
- OpenCV
- librealsense2 SDK

---

## License

See individual package licenses (typically Apache 2.0 and BSD for ROS packages)

---

## Notes

- Ensure your RealSense camera is properly connected and recognized by `realsense-viewer` before running the pipeline
- Camera calibration parameters are recommended for accurate NDVI computation
- See `commandos-ips-serialnumbers.txt` in the ndvi_surco1 directory for device configuration information
