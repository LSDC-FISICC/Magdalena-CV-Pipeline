from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='central_node_pkg',
            executable='central_node',
            name='central_node',
            output='screen',
            parameters=['config/central_params.yaml'],
            # Ejemplo de remapeo, opcional:
            # remappings=[
            #     ('/sensor1', '/cam1/value'),
            #     ('/sensor2', '/cam2/value'),
            #     ('/central_data', '/fusion/value'),
            # ]
        )
    ])
