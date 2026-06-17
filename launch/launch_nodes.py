"""
Launch file for the Fertilizer package.

This launch file orchestrates:
1. n RealSense camera nodes (camera1, camera2, ..., cameraN)
2. n Processing Pipeline nodes (one per camera, cropping ROI)
3. n NDVI Threshold nodes (one per camera, computing NDVI and thresholding)
4. 1 Modbus Controller node (subscribing to /threshold1 through /threshold5)

The number of cameras (n) is read from config/config.yaml (default: 5).
All RealSense parameters (FPS, resolution, serial numbers, etc.) are also from config.yaml.

Topic flow:
  RealSense (cameraX namespace)
    /cameraX/color/raw -> ProcessingPipeline -> /cameraX/roi -> NDVIThreshold -> /ndviX
    /cameraX/infra2/image_rect_raw -> ProcessingPipeline -> /cameraX/infra_roi -> NDVIThreshold
  
  ModbusController (root namespace)
    Subscribes to /threshold1, /threshold2, ..., /threshold5
    Writes to Modbus coils based on threshold values
"""

import os
import yaml
from typing import List, Dict, Any

from launch import LaunchDescription, LaunchContext, Action
from launch.actions import OpaqueFunction, LogInfo
import launch_ros.actions


def load_config(config_path: str) -> Dict[str, Any]:
    """Load configuration from YAML file."""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def launch_setup(context: LaunchContext) -> List[Action]:
    """
    Dynamically generate launch actions based on config.yaml.

    Steps:
    0. Read config.yaml to get n (number of cameras)
    1. Launch n RealSense camera nodes with proper parameters
    2. Launch n ProcessingPipeline nodes remapping inputs/outputs
    3. Launch n NDVIThreshold nodes remapping inputs/outputs
    4. Launch 1 ModbusController node
    """

    # Load configuration
    nodes: List[Action] = []
    user = os.getenv('USER', 'unknown')
    nodes.append(LogInfo(msg=f"[Fertilizer] Starting launch setup for user: {user}"))
    config_path = f'/home/{user}/ros2_ws/src/fertilizer/config/config.yaml'
    config_path_abs = os.path.abspath(config_path)
    try:
        config = load_config(config_path_abs)
    except Exception as e:
        return [LogInfo(msg=f"[FATAL] Error loading config from {config_path_abs}: {e}")]

    # Get number of cameras (default 5)
    n = config.get('n', 5)
    rs_config = config.get('realsense', {})

    if n <= 0:
        return [LogInfo(msg="[ERROR] n must be > 0 in config.yaml")]

    

    # Extract global RealSense parameters
    fps = rs_config.get('fps', 15)
    color_res = rs_config.get('color_res', {})
    color_width = color_res.get('x', 640)
    color_height = color_res.get('y', 360)
    infra_res = rs_config.get('infrared_res', {})
    infra_width = infra_res.get('x', 640)
    infra_height = infra_res.get('y', 360)
    emulate_tty = rs_config.get('emulate_tty', True)
    respawn = rs_config.get('respawn', True)
    respawn_delay = rs_config.get('respawn_delay', 5)

    nodes.append(LogInfo(msg=f"[Fertilizer] Launching {n} camera(s) from config.yaml"))
    nodes.append(LogInfo(msg=f"[Fertilizer] Config path: {config_path_abs}"))

    # =========================================================================
    # STEP 1: Launch n RealSense Camera Nodes
    # =========================================================================
    # Each camera is in its own namespace (camera1, camera2, ..., cameraN).
    # Topics are remapped from the RealSense defaults to:
    #   /cameraX/color/raw (from color/image_raw)
    #   /cameraX/infra2/image_rect_raw (unchanged in namespace)

    nodes.append(LogInfo(msg="[Fertilizer] STEP 1: Launching RealSense cameras..."))

    for i in range(1, n + 1):
        camera_name = f'camera{i}'
        camera_ns = camera_name

        # Get camera-specific serial number
        camera_cfg = rs_config.get(camera_name, {})
        serial = camera_cfg.get('serial', '')

        if not serial:
            nodes.append(
                LogInfo(msg=f"[WARNING] No serial number for {camera_name} in config.yaml")
            )

        # Create RealSense camera node
        camera_node = launch_ros.actions.Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace=camera_ns,
            name=f'camera{i}',
            parameters=[
                {
                    'device_type': 'd555',
                    'serial_no': serial,
                    'rgb_camera.color_profile': f'{color_width}x{color_height}x{fps}',
                    'depth_module.infra_profile': f'{infra_width}x{infra_height}x{fps}',
                    'emulate_tty': emulate_tty,
                    'enable_color': True,
                    'enable_infra1': False,
                    'enable_infra2': True,
                    'enable_depth': False,
                    'depth_module.laser_power': 0.0,
                    'depth_module.emitter_enabled': False,
                }
            ],
            remappings=[
                ('color/image_raw', f'/{camera_ns}/{camera_name}/color/image_raw'),
                ('infra2/image_rect_raw', f'/{camera_ns}/{camera_name}/infra2/image_rect_raw'),
            ],
            respawn=respawn,
            respawn_delay=respawn_delay,
            emulate_tty=emulate_tty,
        )
        nodes.append(camera_node)
        nodes.append(LogInfo(msg=f"  -> {camera_name} (serial: {serial}): color/infra2 enabled, depth enabled but emitter off, laser_power=0"))

    # =========================================================================
    # STEP 2: Launch n Processing Pipeline Nodes
    # =========================================================================
    # Each pipeline node is in the same namespace as its corresponding camera.
    # It subscribes to the camera's color and infra2 topics and publishes:
    #   /cameraX/roi (from /output)
    #   /cameraX/infra_roi (from /infra_output)
    #
    # The pipeline supports an optional alignment matrix (read from
    # config/config.yaml) that warps the infrared image into the color
    # coordinate frame before cropping.  This is useful when using a
    # stock realsense2_camera node, which does not provide IR→color
    # registration.

    nodes.append(LogInfo(msg="[Fertilizer] STEP 2: Launching Processing Pipeline nodes..."))

    for i in range(1, n + 1):
        camera_name = f'camera{i}'
        camera_ns = camera_name

        pipeline_node = launch_ros.actions.Node(
            package='fertilizer',
            executable='processing_pipeline',
            namespace=camera_ns,
            name='processing_pipeline',
            parameters=[
                {
                    'color_topic': f'/{camera_ns}/{camera_name}/color/image_raw',
                    'infra_topic': f'/{camera_ns}/{camera_name}/infra2/image_rect_raw',
                    'color_output_topic': f'/{camera_ns}/{camera_name}/roi',
                    'infra_output_topic': f'/{camera_ns}/{camera_name}/infra_roi',
                }
            ],
            respawn=respawn,
            respawn_delay=respawn_delay,
            emulate_tty=emulate_tty,
        )
        nodes.append(pipeline_node)
        nodes.append(LogInfo(msg=f"  -> {camera_name}/processing_pipeline (camera #{i}):"))
        nodes.append(LogInfo(msg=f"      IN:  /{camera_ns}/{camera_name}/color/image_raw, /{camera_ns}/{camera_name}/infra2/image_rect_raw"))
        nodes.append(LogInfo(msg=f"      OUT: /{camera_ns}/{camera_name}/roi, /{camera_ns}/{camera_name}/infra_roi"))

    # =========================================================================
    # STEP 3: Launch n NDVI Threshold Nodes
    # =========================================================================
    # Each NDVI node is in the same namespace as its corresponding camera.
    # It subscribes to the pipeline outputs and publishes:
    #   /ndviX (at root level, for ModbusController)
    #   /thresholdX (at root level, for monitoring)

    nodes.append(LogInfo(msg="[Fertilizer] STEP 3: Launching NDVI Threshold nodes..."))

    for i in range(1, n + 1):
        camera_name = f'camera{i}'
        camera_ns = camera_name

        ndvi_node = launch_ros.actions.Node(
            package='fertilizer',
            executable='ndvi_threshold',
            namespace=camera_ns,
            name='ndvi_threshold',
            parameters=[
                {
                    'roi_topic': f'/{camera_ns}/{camera_name}/roi',
                    'infra_roi_topic': f'/{camera_ns}/{camera_name}/infra_roi',
                    'threshold_output_topic': f'/threshold{i}',
                    'ndvi_output_topic': f'/{camera_ns}/{camera_name}/ndvi{i}',
                }
            ],
            respawn=respawn,
            respawn_delay=respawn_delay,
            emulate_tty=emulate_tty,
        )
        nodes.append(ndvi_node)
        nodes.append(LogInfo(msg=f"  -> {camera_name}/ndvi_threshold (camera #{i}):"))
        nodes.append(LogInfo(msg=f"      IN:  /{camera_ns}/{camera_name}/roi (from pipeline), /{camera_ns}/{camera_name}/infra_roi (from pipeline)"))
        nodes.append(LogInfo(msg=f"      OUT: /threshold{i} (to modbus controller), /{camera_ns}/{camera_name}/ndvi{i} (monitoring)"))

    # =========================================================================
    # STEP 4: Launch 1 Modbus Controller Node
    # =========================================================================
    # The ModbusController subscribes to /ndvi1 through /ndvi5.
    # If n < 5, it will have unused subscriptions (acceptable per requirements).
    # It writes NDVI results to Modbus coils 0-4.

    nodes.append(LogInfo(msg="[Fertilizer] STEP 4: Launching Modbus Controller..."))

    modbus_node = launch_ros.actions.Node(
        package='fertilizer',
        executable='modbus_controller',
        name='modbus_controller',
        respawn=respawn,
        respawn_delay=respawn_delay,
        emulate_tty=emulate_tty,
    )
    nodes.append(modbus_node)
    nodes.append(LogInfo(msg=f"  -> modbus_controller: subscribes to /threshold1 through /threshold{n}, writes to Modbus coils 0-{n-1}"))

    nodes.append(
        LogInfo(msg=f"[Fertilizer] Launch complete: {n} cameras + pipelines + ndvi + modbus")
    )

    return nodes


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description."""
    
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup)
        ]
    )
