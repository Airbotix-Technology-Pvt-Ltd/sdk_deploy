import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('isaac_bridge'),
        'config',
        'bridge_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='isaac_bridge',
            executable='state_bridge_node',
            name='state_bridge_node',
            output='screen',
            parameters=[config, {'use_sim_time': True}]
        ),
        Node(
            package='isaac_bridge',
            executable='cmd_bridge_node',
            name='cmd_bridge_node',
            output='screen',
            parameters=[config, {'use_sim_time': True}]
        )
    ])
