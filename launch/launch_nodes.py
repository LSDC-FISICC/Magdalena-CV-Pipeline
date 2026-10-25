from launch import LaunchDescription , LaunchContext, Action
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from typing import List, Dict, Any
import yaml
import os


def load_config(config_path: str) -> Dict[str, Any]:
    """Load configuration from YAML file."""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description() : 

    nodes = []
    # ----------------------------------------------------------------------
    # Configuration Path & File Parsing
    # ----------------------------------------------------------------------
    user = os.getenv('USER', 'unknown')
    config_path = f'./src/fertilizer/config/config.yaml'
    config_path_abs = os.path.abspath(config_path)
    try:
        config = load_config(config_path_abs)
    except Exception as e:
        return LaunchDescription([LogInfo(msg=f"[FATAL] Error loading config from {config_path_abs}: {e}")])

    # Get number of cameras/Coil (default 5)
    n = config.get('n', 5)
    rs_config = config.get('realsense', {})

    if n <= 0:
        return LaunchDescription([LogInfo(msg="[ERROR] n must be > 0 in config.yaml")])

    
    # ----------------------------------------------------------------------
    # Global RealSense Stream Parameters
    # ----------------------------------------------------------------------

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



    for i in range(1, n + 1):
        camera_name = f'camera{i}'
        camera_ns = camera_name

        # Get camera-specific serial number
        camera_cfg = rs_config.get(camera_name, {})
        serial = camera_cfg.get('serial', '')

        #Create the Node for each Cameras
        camera = Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace=camera_ns,
            name=f'camera{i}',
            respawn=False,
            parameters=[
                {
                    'device_type': 'd555',
                    'serial_no': serial,
                    'wait_for_device_timeout': 5.0,  #Timeout connection attempt after 5 seconds (used to don't stop the programm if some cameras are not connected)
                    'reconnect_timeout': 5.0,  
                    'rgb_camera.color_profile': f'{color_width}x{color_height}x{fps}',
                    'depth_module.infra_profile': f'{infra_width}x{infra_height}x{fps}',
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
        )
        nodes.append(camera)

    # ----------------------------------------------------------------------
    # Core System Processing, Inference & Controller Nodes
    # ----------------------------------------------------------------------

    # Image preprocessing worker node
    cameraNode = Node(
        package = 'cv_inference', 
        executable = 'cameraNode', 
        name = 'camera_process_node',
        output = 'screen',
        respawn=respawn,
        respawn_delay=respawn_delay,
        emulate_tty=emulate_tty,
        #additional_env={'LD_PRELOAD': '/lib/x86_64-linux-gnu/libpthread.so.0'},
    )

    # YOLO AI Object Detection Inference node
    inference_node = Node(
        package = 'cv_inference', 
        executable = 'inference_node', 
        name = 'Yolo_InferenceNode_v2',
        output = 'screen',
        respawn=respawn,
        respawn_delay=respawn_delay,
        emulate_tty=emulate_tty,
    )

    # Valves controller Node
    fertilizer_node = Node(
        package = 'fertilizer', 
        executable = 'modbus_controller', 
        name = 'modbus_controller', 
        output = 'screen',
        respawn=respawn,
        respawn_delay=respawn_delay,
        emulate_tty=emulate_tty,
    )
    # Append core infrastructure to the execution stack
    nodes.append(cameraNode)
    nodes.append(inference_node)
    nodes.append(fertilizer_node)

    return LaunchDescription(nodes)
