import os
import yaml
import copy
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchContext
import launch_ros.actions
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import pathlib
sys.path.append('/home/orin30/ros2_ws/src/ndvi_surco1/launch')
import rs_launch


local_parameters = [{'name': 'camera_name1', 'default': 'camera55', 'description': 'camera55 unique name'},
                    {'name': 'camera_name2', 'default': 'camera56', 'description': 'camera56 unique name'},
                    {'name': 'camera_name3', 'default': 'camera57', 'description': 'camera57 unique name'},
                    {'name': 'camera_name4', 'default': 'camera58', 'description': 'camera58 unique name'},
                    {'name': 'camera_name5', 'default': 'camera59', 'description': 'camera59 unique name'},
                    {'name': 'camera_namespace1', 'default': 'camera55', 'description': 'camera55 namespace'},
                    {'name': 'camera_namespace2', 'default': 'camera56', 'description': 'camera56 namespace'},
                    {'name': 'camera_namespace3', 'default': 'camera57', 'description': 'camera57 namespace'},
                    {'name': 'camera_namespace4', 'default': 'camera58', 'description': 'camera58 namespace'},
                    {'name': 'camera_namespace5', 'default': 'camera59', 'description': 'camera59 namespace'},
                    {'name': 'serial_no1', 'default': "'419222301921'", 'description': 'Serial number of camera55'},
                    {'name': 'serial_no2', 'default': "'353522302286'", 'description': 'Serial number of camera56'},
                    {'name': 'serial_no3', 'default': "'409122301544'", 'description': 'Serial number of camera57'},
                    {'name': 'serial_no4', 'default': "'408222300679'", 'description': 'Serial number of camera58'},
                    {'name': 'serial_no5', 'default': "'353522301659'", 'description': 'Serial number of camera59'},
                   
                    ]

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params

def launch_static_transform_publisher_node(context : LaunchContext):
    # dummy static transformation from camera1 to camera2
    node = launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0",
                          context.launch_configurations['camera_name1'] + "_link",
                          context.launch_configurations['camera_name2'] + "_link",
                          context.launch_configurations['camera_name3'] + "_link",
                          context.launch_configurations['camera_name4'] + "_link",
                          context.launch_configurations['camera_name5'] + "_link"]
    )
    return [node]


def launch_setup(context, params, param_name_suffix=''):
    _config_file = LaunchConfiguration('config_file' + param_name_suffix).perform(context)
    params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)

    _output = LaunchConfiguration('output' + param_name_suffix)
    if(os.getenv('ROS_DISTRO') == 'foxy'):
        # Foxy doesn't support output as substitution object (LaunchConfiguration object)
        # but supports it as string, so we fetch the string from this substitution object
        # see related PR that was merged for humble, iron, rolling: https://github.com/ros2/launch/pull/577
        _output = context.perform_substitution(_output)

    return [
        launch_ros.actions.Node(
            package='realsense2_camera',
            namespace=LaunchConfiguration('camera_namespace' + param_name_suffix),
            name=LaunchConfiguration('camera_name' + param_name_suffix),
            executable='realsense2_camera_node',
            parameters=[params, params_from_file],
            output=_output,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level' + param_name_suffix)],
            emulate_tty=True,
            # respawn=True,
            # respawn_delay=0.5
            )
    ]

def generate_launch_description():
    
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')
    params3 = duplicate_params(rs_launch.configurable_parameters, '3')
    params4 = duplicate_params(rs_launch.configurable_parameters, '4')
    params5 = duplicate_params(rs_launch.configurable_parameters, '5')
    # --- Nodo para Cámara 1 ---
    node_cam1 = Node(
        package='ndvi_surco1',
        executable='nocheyndvi',
        name='noche_ndvi_1',     # Nombre único para evitar conflictos
        output='screen',
        parameters=[
            {'ad': 0}  # <-- Define el parámetro 'ad' con el valor entero 1
        ],
        remappings=[
            ('image',  '/camera55/camera55/color/image_raw'),
            ('image2', '/camera55/camera55/infra2/image_rect_raw'),
            ('out',    '/camera55/proceso'),
            ('out2',   '/camera55/estado')
        ]
    )

    # --- Nodo para Cámara 2 ---
    node_cam2 = Node(
        package='ndvi_surco1',
        executable='nocheyndvi',
        name='noche_ndvi_2',
        output='screen',
        parameters=[
            {'ad': 1}  # <-- Define el parámetro 'ad' con el valor entero 1
        ],
        remappings=[
            ('image',  '/camera56/camera56/color/image_raw'),
            ('image2', '/camera56/camera56/infra2/image_rect_raw'),
            ('out',    '/camera56/proceso'),
            ('out2',   '/camera56/estado')
        ]
    )
    node_cam3 = Node(
        package='ndvi_surco1',
        executable='nocheyndvi',
        name='noche_ndvi_3',
        output='screen',
        parameters=[
            {'ad': 2}  # <-- Define el parámetro 'ad' con el valor entero 1
        ],
        remappings=[
            ('image',  '/camera57/camera57/color/image_raw'),
            ('image2', '/camera57/camera57/infra2/image_rect_raw'),
            ('out',    '/camera57/proceso'),
            ('out2',   '/camera57/estado')
        ]
    )
    # --- Nodo para Cámara 4 ---
    node_cam4 = Node(
        package='ndvi_surco1',
        executable='nocheyndvi',
        name='noche_ndvi_4',
        output='screen',
        parameters=[
            {'ad':3}
        ],
        remappings=[
            ('image',  '/camera58/camera58/color/image_raw'),
            ('image2', '/camera58/camera58/infra2/image_rect_raw'),
            ('out',    '/camera58/proceso'),
            ('out2',   '/camera58/estado')
        ]
    )
    node_cam5 = Node(
        package='ndvi_surco1',
        executable='nocheyndvi',
        name='noche_ndvi_5',
        output='screen',
        parameters=[
            {'ad':4}
        ],
        remappings=[
            ('image',  '/camera59/camera59/color/image_raw'),
            ('image2', '/camera59/camera59/infra2/image_rect_raw'),
            ('out',    '/camera59/proceso'),
            ('out2',   '/camera59/estado')
        ]
    )

   
    # ndvii = Node(
    #     package='ndvi_mag',
    #     executable='ndvi',
    #     name='ndvii',
    #     output='screen',
    #      # respawn=True,
    #      # respawn_delay=1.0
    # )
    # ndvii2 = Node(
    #     package='ndvi_mag2',
    #     executable='ndvii',
    #     name='ndvii',
    #     output='screen',
    #      # respawn=True,
    #      # respawn_delay=1.0
    # )
    

    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
         rs_launch.declare_configurable_parameters(params1) +
        # rs_launch.declare_configurable_parameters(params2) +
        rs_launch.declare_configurable_parameters(params3) +
        # rs_launch.declare_configurable_parameters(params4) +
        # rs_launch.declare_configurable_parameters(params5) +
        
        [
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params1),
                                 'param_name_suffix': '1'}),
        # OpaqueFunction(function=rs_launch.launch_setup,
        #                kwargs = {'params'           : set_configurable_parameters(params2),
        #                          'param_name_suffix': '2'}),
        # OpaqueFunction(function=rs_launch.launch_setup,
        #                kwargs = {'params'           : set_configurable_parameters(params4),
        #                          'param_name_suffix': '5'}),
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params3),
                                 'param_name_suffix': '3'}),
        # OpaqueFunction(function=rs_launch.launch_setup,
        #                 kwargs = {'params'           : set_configurable_parameters(params4),
        #                           'param_name_suffix': '4'}),

        # OpaqueFunction(function=rs_launch.launch_setup,
        #                 kwargs = {'params'           : set_configurable_parameters(params5),
        #                           'param_name_suffix': '5'}),
        
        OpaqueFunction(function=launch_static_transform_publisher_node),
        node_cam1,
        #node_cam2,
        node_cam3,
        #node_cam4,
        #node_cam5
        ]
      
        
    )

