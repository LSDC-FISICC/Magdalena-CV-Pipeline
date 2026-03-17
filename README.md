# Fertilizer Control System

A ROS2 package for automated fertilizer application control using multiple RealSense D555 cameras to detect plant presence and block/allow fertilizer flow accordingly.

## Overview

This system uses computer vision to monitor crop rows via infrared and color imaging from multiple RealSense cameras. It automatically controls fertilizer application by calculating the Normalized Difference Vegetation Index (NDVI) to determine plant presence, then sending control signals via Modbus TCP to fertilizer distribution equipment.

## Features

- **Multi-camera Support**: Handles up to 5+ simultaneous RealSense D555 cameras
- **NDVI-based Plant Detection**: Computes NDVI (Normalized Difference Vegetation Index) from infrared and color images
- **Alternative Index Support**: Can also use EGX (Excess Green Index) for testing
- **Modbus TCP Control**: Interfaces with fertilizer control hardware via Modbus TCP
- **ROI Processing**: Crops and processes specific Regions of Interest (ROI) from camera feeds
- **Dynamic Configuration**: All parameters configurable via YAML without recompilation
- **Automatic Respawning**: Nodes automatically respawn on failure

## System Architecture

The system follows a ROS2 nodelet pattern with the following data flow:

```
RealSense Camera (Color + Infrared)
    ↓
Processing Pipeline (ROI Cropping)
    ↓
NDVI Threshold (Index Calculation & Thresholding)
    ↓
Modbus Controller (Hardware Control)
    ↓
Fertilizer Equipment
```

### Key Nodes

1. **RealSense Camera Nodes** (`realsense2_camera`)
   - Publishes color and infrared image streams
   - One node per connected camera

2. **Processing Pipeline Node** (`processing_pipeline`)
   - Subscribes to color and infrared images
   - Crops Region of Interest (ROI) based on config
   - Supports optional infrared-to-color alignment (static or dynamic via RANSAC)
   - Publishes cropped `/roi` and `/infra_roi` topics

3. **NDVI Threshold Node** (`ndvi_threshold`)
   - Subscribes to cropped color and infrared images
   - Computes NDVI or EGX index
   - Applies threshold to determine plant presence
   - Publishes selected Index value in `/threshold` topic
   - Publishes NDVI visualization as colormap image

4. **Modbus Controller Node** (`modbus_controller`)
   - Subscribes to all `/threshold1` through `/threshold5` topics
   - Writes control commands to Modbus TCP device based on index


## Installation

### Prerequisites

- Ubuntu 20.04 LTS or later
- ROS2 Humble
- Python 3.8+

### Dependencies

Install system dependencies:
```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep install -i --from-path src --rosdistro humble -y
```

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select fertilizer --symlink-install
source install/setup.bash
```

## Configuration

Edit `config/config.yaml` to customize params.

## Running

### Launch All Nodes

```bash
ros2 launch fertilizer launch_nodes.py
```

This automatically launches:
- All configured RealSense cameras
- Processing pipeline for each camera
- NDVI threshold node for each camera
- Single Modbus controller node

### Run Individual Nodes

```bash
# Processing pipeline
ros2 run fertilizer processing_pipeline

# NDVI threshold computation
ros2 run fertilizer ndvi_threshold

# Modbus controller
ros2 run fertilizer modbus_controller
```

## Topics

### Input Topics (RealSense)
- `/camera1/color/raw` (sensor_msgs/Image) - Color image
- `/camera1/infra2/image_rect_raw` (sensor_msgs/Image) - Infrared image

### Processing Topics
- `/camera1/roi` (sensor_msgs/Image) - Cropped color image
- `/camera1/infra_roi` (sensor_msgs/Image) - Cropped infrared image

### Output Topics
- `/ndvi1` (std_msgs/Float32) - NDVI value [0.0, 1.0]
- `/ndvi` (sensor_msgs/Image) - NDVI visualization with colormap
- `/threshold1` (std_msgs/Float32) - Binary decision (0.0 or 1.0)

## Project Structure

```
fertilizer/
├── config/
│   └── config.yaml              # Configuration file
├── fertilizer/
│   ├── __init__.py
│   ├── modbus_controller.py     # Modbus TCP interface node
│   ├── ndvi_threshold.py        # NDVI calculation & thresholding
│   ├── processing_pipeline.py   # ROI extraction & alignment
│   └── __pycache__/
├── launch/
│   └── launch_nodes.py          # ROS2 launch file
├── resource/
│   └── fertilizer               # Package marker
├── test/
│   ├── test_copyright.py        # Copyright test
│   ├── test_flake8.py           # Code style test
│   └── test_pep257.py           # Docstring test
├── LICENSE                      # GPL-3.0-only
├── package.xml                  # ROS2 package descriptor
├── README.md                    # This file
├── setup.cfg                    # Setup configuration
└── setup.py                     # Package setup script
```

## Module Details

### processing_pipeline.py

Handles image preprocessing:
- Subscribes to RealSense color and infrared streams
- Applies optional infrared-to-color alignment (static or dynamic)
- Crops a Region of Interest (ROI) based on configuration
- Publishes cropped images for downstream processing
- Uses approximate time synchronization to align camera frames

Key function:
- `get_roi_dimensions()` - Calculates ROI size in pixels based on camera height and physical dimensions using trigonometry

### ndvi_threshold.py

Computes vegetation indices and thresholding:
- Subscribes to cropped color and infrared images
- Calculates NDVI: `(NIR - RED) / (NIR + RED)` or EGX: `(2 * GREEN) - (RED + BLUE)`
- Applies configurable threshold
- Publishes binary decision (1.0 if vegetation detected, 0.0 otherwise)
- Publishes NDVI/EGX as colormap visualization (RdYlGn-like)

### modbus_controller.py

Modbus TCP interface:
- Subscribes to `/threshold1` through `/threshold5` topics
- Maintains persistent connection to Modbus TCP server
- Writes strictly binary values (0.0 or 1.0) to coils 0-4
- Handles connection errors gracefully
- Loads configuration from YAML with fallback paths

## Dependencies

### ROS2 Packages
- `rclpy` - ROS2 Python client library
- `std_msgs` - Standard message types
- `sensor_msgs` - Image message types
- `cv_bridge` - OpenCV/ROS bridge
- `launch` - ROS2 launch system
- `launch_ros` - ROS2 launch utilities
- `realsense2_camera` - RealSense driver

### Python Packages
- `opencv_python` - Computer vision library
- `numpy` - Numerical computing
- `pyyaml` - YAML configuration
- `pyModbusTCP` - Modbus TCP client library
- `message_filters` - ROS2 message synchronization

### Testing
- `pytest` - Testing framework
- `ament_copyright` - Copyright checker
- `ament_flake8` - PEP8 style linter
- `ament_pep257` - Docstring checker

## Testing

Run tests:
```bash
cd ~/ros2_ws
colcon test --packages-select fertilizer
```

## License

GPL-3.0-only

See LICENSE file for details.

## Maintainer

LSDC (lsdc@galileo.edu)
