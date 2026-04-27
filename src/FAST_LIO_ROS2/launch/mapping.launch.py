import os.path
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_path').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    full_config_path = os.path.join(config_path, config_file)

    # Default topics and enable status (OFF by default)
    cloud_in = '/cloud_registered_body'
    scan_out = '/scan'
    is_enabled = False

    # Try to read topics and enabled flag from YAML
    if os.path.exists(full_config_path):
        try:
            with open(full_config_path, 'r') as f:
                data = yaml.safe_load(f)
                root_params = data.get('/**', {}).get('ros__parameters', {})
                # Extract the entire nested block
                nested_params = root_params.get('pointcloud_to_laserscan', {})
                
                # Update remappings and enable flag from nested if present
                # Default fallback logic if keys are missing
                cloud_in = nested_params.get('cloud_in', root_params.get('cloud_in', '/cloud_registered_body'))
                scan_out = nested_params.get('scan_out', root_params.get('scan_out', '/scan'))
                is_enabled = nested_params.get('enabled', root_params.get('enabled', is_enabled))
        except Exception:
            pass

    # Launch toggle (cli argument takes precedence if provided, but here we merge)
    # If laserscan:=false is passed, it override YAML enabled: true.
    laserscan_launch_arg = LaunchConfiguration('laserscan').perform(context).lower() == 'true'
    should_run = is_enabled and laserscan_launch_arg

    pc2_to_ls_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[('cloud_in', cloud_in),
                    ('scan', scan_out)],
        parameters=[nested_params, {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition('true' if should_run else 'false'),
        output='screen'
    )

    return [pc2_to_ls_node]


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    # Argument to override YAML enabled flag
    declare_laserscan_cmd = DeclareLaunchArgument(
        'laserscan', default_value='true',
        description='Enable pointcloud_to_laserscan (overrides YAML enabled flag if false)'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_laserscan_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)
    
    # Dynamic setup for pointcloud_to_laserscan
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
