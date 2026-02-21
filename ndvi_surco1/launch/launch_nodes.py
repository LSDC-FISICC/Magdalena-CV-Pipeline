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
# make the package's launch directory importable regardless of current working dir
sys.path.append(str(pathlib.Path(__file__).resolve().parent))
import rs_launch


local_parameters = [{'name': 'camera_name1', 'default': 'camera1', 'description': 'camera1 unique name'},
                    {'name': 'camera_name2', 'default': 'camera2', 'description': 'camera2 unique name'},
                    {'name': 'camera_name3', 'default': 'camera3', 'description': 'camera3 unique name'},
                    {'name': 'camera_namespace1', 'default': 'camera1', 'description': 'camera1 namespace'},
                    {'name': 'camera_namespace2', 'default': 'camera2', 'description': 'camera2 namespace'},
                    {'name': 'camera_namespace3', 'default': 'camera3', 'description': 'camera3 namespace'},
                    {'name': 'serial_no1', 'default': "'419222301921'", 'description': 'Serial number of camera1'},
                    {'name': 'serial_no2', 'default': "'353522302286'", 'description': 'Serial number of camera2'},
                    {'name': 'serial_no3', 'default': "'409122301544'", 'description': 'Serial number of camera3'},
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
                          context.launch_configurations['camera_name2'] + "_link"]
    )
    return [node]


def launch_setup(context, params, param_name_suffix=''):
    _config_file = LaunchConfiguration('config_file' + param_name_suffix).perform(context)
    params_from_file = {} if _config_file == "''" else rs_launch.yaml_to_dict(_config_file)

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
            respawn=True,
            respawn_delay=0.5
            )
    ]

def generate_launch_description():
    
   
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')
    params3 = duplicate_params(rs_launch.configurable_parameters, '3')
    my_custom_node = Node(
        package='ndvi_surco1',
        executable='nocheyndvi',
        name='ndvi1',
        output='screen',
        parameters=[{
            'ad': 1,
            'camera_rgb_topic': '/camera1/camera1/color/image_raw',
            'camera_infra_topic': '/camera1/camera1/infra2/image_rect_raw',
            'data_topic': '/camera1/proceso',
            'data_topic2': '/camera1/estado',
        }],
    )
    my_custom_node2 = Node(
        package='ndvi_surco1',
        executable='nocheyndvi',
        name='ndvi2',
        output='screen',
        parameters=[{
            'ad': 2,
            'camera_rgb_topic': '/camera2/camera2/color/image_raw',
            'camera_infra_topic': '/camera2/camera2/infra2/image_rect_raw',
            'data_topic': '/camera2/proceso',
            'data_topic2': '/camera2/estado',
        }],
    )
    my_custom_node3 = Node(
        package='ndvi_surco1',
        executable='nocheyndvi',
        name='ndvi3',
        output='screen',
        parameters=[{
            'ad': 3,
            'camera_rgb_topic': '/camera3/camera3/color/image_raw',
            'camera_infra_topic': '/camera3/camera3/infra2/image_rect_raw',
            'data_topic': '/camera3/proceso',
            'data_topic2': '/camera3/estado',
        }],
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
        rs_launch.declare_configurable_parameters(params2) +
        rs_launch.declare_configurable_parameters(params3) +
         [
        OpaqueFunction(function=launch_setup,
                   kwargs = {'params'           : set_configurable_parameters(params1),
                     'param_name_suffix': '1'}),
        OpaqueFunction(function=launch_setup,
                   kwargs = {'params'           : set_configurable_parameters(params2),
                     'param_name_suffix': '2'}),
        OpaqueFunction(function=launch_setup,
                   kwargs = {'params'           : set_configurable_parameters(params3),
                     'param_name_suffix': '3'}),
        # static transform between cameras (optional)
        # OpaqueFunction(function=launch_static_transform_publisher_node),
        my_custom_node, my_custom_node2, my_custom_node3]
      
        
    )

