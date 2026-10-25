# Fertilizer Control System

A ROS2 package for automated fertilizer application control using multiple Intel RealSense D555 cameras to detect plant presence and block/allow fertilizer flow accordingly.

## Overview

This system uses computer vision and deep learning (YOLO) to monitor crop rows via multiple RealSense cameras. It automatically controls fertilizer application by analyzing incoming plant detection triggers and sending hardware control commands via Modbus TCP to physical distribution valves.

## Features

- **Multi-camera Support**: Dynamically handles up to 5+ simultaneous RealSense D555 cameras based on configuration.
- **YOLO-Driven Detection**: Integrates with a specialized inference pipeline for robust plant detection.
- **Modbus TCP Control**: Interfaces with industrial fertilizer distribution hardware via `pyModbusTCP`.
- **Hardware Protection (Debouncing)**: Features a software-level debounce mechanism (configured in microseconds) to protect physical valves from rapid, damaging state changes.
- **Robust Configuration Fallbacks**: The controller automatically searches multiple path strategies to successfully locate its YAML configuration file across different environments (Jetson, Docker, local workspace).
- **Automatic Respawning**: Critical camera nodes are configured within the launch system to automatically recover on failure.
- **Preprocess input images**: Preprocess input image to adapt them for the Yolo arhcitecture

## System Architecture

Intel RealSense D555 Cameras (Color + Infrared Streams)
↓
cv_inference / camera_process_node (Image Preprocessing)
↓
cv_inference / Yolo_InferenceNode_v2 (AI Object Detection)
↓  [Published onto /detection/Trigger as DiagnosticArray]
fertilizer / modbus_controller (Debounce Check & Register Mapping)
↓  [Modbus TCP Commands]
Physical Fertilizer Valves / Equipment

### Key Nodes

1. **Modbus Controller Node** (`modbus_controller`)
   - Subscribes to the global `/detection/Trigger` topic.
   - Parses the incoming `DiagnosticArray` to find `plant_detected` boolean statuses.
   - Maps camera namespaces to target Modbus coils dynamically using `config.yaml`.
   - Safely forces all fertilizer channels open (`True`) at startup to ensure baseline flow.
   - Applies strict timing checks against `debounce_time` before letting a valve toggle.

*Note: Camera drivers (`realsense2_camera_node`) and vision pipelines (`cameraNode`, `inference_node`) are managed by their respective packages but are fully orchestrated by this package's launch configuration.*


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
ros2 launch fertilizer launch_nodes.py                      # perception from config.yaml (default: yolo)
ros2 launch fertilizer launch_nodes.py perception:=index    # NDVI / ExG index masks only
ros2 launch fertilizer launch_nodes.py perception:=both valve_source:=yolo
```

This launches all configured RealSense cameras, one single Modbus controller node and the
perception pipeline selected by `perception`:

| `perception` | Nodes | Decisions |
|---|---|---|
| `yolo` | `cv_inference` `cameraNode` + `inference_node` (YOLO26 classifier, one batch of all cameras) | `/detection/Trigger` |
| `index` | `processing_pipeline` + `ndvi_threshold` per camera (`index: ndvi` or `egx` in config.yaml) | `/threshold<i>` |
| `both` | both pipelines on the same frames | both topics |

With `both`, `valve_source` (`yolo` or `index`) chooses the pipeline that drives the valves;
the other one only publishes, so the two can be logged and compared on identical frames.

**Cameras.** On the Jetson (reComputer J3011) the five D555 are connected over USB 3.2 and
driven by `realsense2_camera`. If they fail to connect when started together, set
`realsense.startup_stagger_s` in config.yaml (e.g. 5.0): camera *i* then starts
`(i-1) * startup_stagger_s` seconds after launch. The default, 0, starts them all at once.
Over PoE the J3011 network interface cannot keep up with five streams; PoE with the
`magdalene_camera` driver (Magdalena-Camera repository) works on a laptop-class NIC.

### Run Individual Nodes

```bash
# Modbus controller
ros2 run fertilizer modbus_controller
```

## Topics for the code of this specific package for cameraNode and Inference_Node go check cv_inference

### Input Topics (CameraNode)
- `/detection/Trigger` (diagnostic_msgs/DiagnosticArray) - Contains statuses for active camera streams. The node scans for a key named plant_detected matching a string value of "true" or "false".

### Output Topics
- `Modbus TCP Coils` (boolean) - Direct boolean writes (True / False) over the network to the configured host and port.

## Project Structure

```
fertilizer/
├── config/
│   └── config.yaml              # Configuration file
├── fertilizer/
│   ├── __init__.py
│   ├── modbus_controller.py     # Modbus TCP interface node
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
