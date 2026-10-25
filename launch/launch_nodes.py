from launch import LaunchDescription , LaunchContext, Action
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from typing import List, Dict, Any
import yaml
import os


def load_config(config_path: str) -> Dict[str, Any]:
    """Load configuration from YAML file."""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def launch_setup(context: LaunchContext) -> List[Action]:
    """Build the node list for the perception pipeline chosen by `perception`.

    perception (launch argument, else config.yaml):
      yolo  -> cv_inference cameraNode + inference_node
      index -> processing_pipeline + ndvi_threshold per camera (NDVI or ExG masks)
      both  -> both; valve_source picks the one that drives the valves, the other
               only publishes its decisions so both can be logged on the same frames
    """
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
        return [LogInfo(msg=f"[FATAL] Error loading config from {config_path_abs}: {e}")]

    # Get number of cameras/Coil (default 5)
    n = config.get('n', 5)
    rs_config = config.get('realsense', {})

    if n <= 0:
        return [LogInfo(msg="[ERROR] n must be > 0 in config.yaml")]

    perception = LaunchConfiguration('perception').perform(context) or config.get('perception', 'yolo')
    valve_source = LaunchConfiguration('valve_source').perform(context) or config.get('valve_source', perception)
    if perception not in ('yolo', 'index', 'both'):
        return [LogInfo(msg=f"[ERROR] perception must be yolo, index or both (got {perception})")]
    if perception != 'both':
        valve_source = perception  # only one pipeline runs: it drives the valves
    if valve_source not in ('yolo', 'index'):
        return [LogInfo(msg=f"[ERROR] valve_source must be yolo or index (got {valve_source})")]
    nodes.append(LogInfo(msg=f"[Fertilizer] perception={perception}, valves driven by {valve_source}"))

    
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
    # Optional seconds between camera starts (USB 3.2 cameras that fail to connect when
    # started together); 0, the default, starts them all at once.
    stagger = float(rs_config.get('startup_stagger_s', 0.0))



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
        # Camera i starts (i-1)*stagger seconds after launch
        nodes.append(TimerAction(period=(i - 1) * stagger, actions=[camera]) if stagger > 0 and i > 1 else camera)

    # ----------------------------------------------------------------------
    # YOLO26 pipeline: one batch of all cameras -> /detection/Trigger
    # ----------------------------------------------------------------------
    if perception in ('yolo', 'both'):
        # Image preprocessing worker node
        nodes.append(Node(
            package = 'cv_inference',
            executable = 'cameraNode',
            name = 'camera_process_node',
            output = 'screen',
            # Batch order = furrow order; the camera name in each topic (cameraN) selects
            # the coil through modbus.camera_map.
            parameters=[{'camera_topics': [f'/camera{i}/camera{i}/color/image_raw' for i in range(1, n + 1)]}],
            respawn=respawn,
            respawn_delay=respawn_delay,
            emulate_tty=emulate_tty,
            #additional_env={'LD_PRELOAD': '/lib/x86_64-linux-gnu/libpthread.so.0'},
        ))
        # YOLO AI classification inference node
        nodes.append(Node(
            package = 'cv_inference',
            executable = 'inference_node',
            name = 'Yolo_InferenceNode_v2',
            output = 'screen',
            parameters=[{'max_batch_size': n}],
            respawn=respawn,
            respawn_delay=respawn_delay,
            emulate_tty=emulate_tty,
        ))

    # ----------------------------------------------------------------------
    # Index pipeline: ROI crop + NDVI/ExG threshold per camera -> /threshold<i>
    # ----------------------------------------------------------------------
    if perception in ('index', 'both'):
        for i in range(1, n + 1):
            camera_name = f'camera{i}'
            camera_ns = camera_name
            nodes.append(Node(
                package='fertilizer',
                executable='processing_pipeline',
                namespace=camera_ns,
                name='processing_pipeline',
                parameters=[{
                    'color_topic': f'/{camera_ns}/{camera_name}/color/image_raw',
                    'infra_topic': f'/{camera_ns}/{camera_name}/infra2/image_rect_raw',
                    'color_output_topic': f'/{camera_ns}/{camera_name}/roi',
                    'infra_output_topic': f'/{camera_ns}/{camera_name}/infra_roi',
                }],
                respawn=respawn,
                respawn_delay=respawn_delay,
                emulate_tty=emulate_tty,
            ))
            nodes.append(Node(
                package='fertilizer',
                executable='ndvi_threshold',
                namespace=camera_ns,
                name='ndvi_threshold',
                parameters=[{
                    'roi_topic': f'/{camera_ns}/{camera_name}/roi',
                    'infra_roi_topic': f'/{camera_ns}/{camera_name}/infra_roi',
                    'threshold_output_topic': f'/threshold{i}',
                    'ndvi_output_topic': f'/{camera_ns}/{camera_name}/ndvi{i}',
                }],
                respawn=respawn,
                respawn_delay=respawn_delay,
                emulate_tty=emulate_tty,
            ))

    # ----------------------------------------------------------------------
    # Valves controller node: listens to the pipeline chosen by valve_source
    # ----------------------------------------------------------------------
    nodes.append(Node(
        package = 'fertilizer',
        executable = 'modbus_controller',
        name = 'modbus_controller',
        output = 'screen',
        parameters=[{'source': valve_source}],
        respawn=respawn,
        respawn_delay=respawn_delay,
        emulate_tty=emulate_tty,
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('perception', default_value='',
                              description='yolo | index | both (default: config.yaml)'),
        DeclareLaunchArgument('valve_source', default_value='',
                              description='yolo | index, used with perception:=both (default: config.yaml)'),
        OpaqueFunction(function=launch_setup),
    ])
