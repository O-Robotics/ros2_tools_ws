#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='bag_config.yaml',
        description='YAML config file name (bag_config.yaml, compressed_config.yaml, extended_sensors_config.yaml)'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='',
        description='Override output directory (leave empty to use config file value)'
    )
    
    # Get the config file path
    config_file_path = PathJoinSubstitution([
        FindPackageShare('bag_recorder'),
        'config',
        LaunchConfiguration('config_file')
    ])
    
    # Create the bag recorder node with YAML parameters
    bag_recorder_node = Node(
        package='bag_recorder',
        executable='bag_recorder_node.py',
        name='bag_recorder',
        output='screen',
        parameters=[
            config_file_path,
            {
                'output_dir': LaunchConfiguration('output_dir')
            }
        ],
        # Only override output_dir if provided
        condition=lambda context: True
    )
    
    # Log info about the recording
    log_info = LogInfo(
        msg=[
            'Starting sensor data bag recording with YAML config...\n',
            'Config file: ', LaunchConfiguration('config_file'), '\n',
            'Press Ctrl+C to stop recording'
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        output_dir_arg,
        log_info,
        bag_recorder_node,
    ])
