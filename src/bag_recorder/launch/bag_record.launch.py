#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='~/ros2_bags',
        description='Directory to save bag files'
    )
    
    bag_name_arg = DeclareLaunchArgument(
        'bag_name',
        default_value='',
        description='Name of the bag file (timestamp will be added if empty)'
    )
    
    topics_arg = DeclareLaunchArgument(
        'topics',
        default_value='["/navigation_layer/odometry/global", "/navigation_layer/odometry/local", "/hardware_layer/diff_cont/odom", "/hardware_layer/imu/data_raw", "/navsat", "/hardware_layer/joint_states"]',
        description='List of topics to record'
    )
    
    max_duration_arg = DeclareLaunchArgument(
        'max_duration',
        default_value='0',
        description='Maximum recording duration in seconds (0 for unlimited)'
    )
    
    max_size_arg = DeclareLaunchArgument(
        'max_size',
        default_value='0',
        description='Maximum bag size in MB (0 for unlimited)'
    )
    
    compression_mode_arg = DeclareLaunchArgument(
        'compression_mode',
        default_value='none',
        description='Compression mode: none, file, message'
    )
    
    compression_format_arg = DeclareLaunchArgument(
        'compression_format',
        default_value='zstd',
        description='Compression format: zstd, lz4'
    )
    
    storage_format_arg = DeclareLaunchArgument(
        'storage_format',
        default_value='sqlite3',
        description='Storage format: sqlite3, mcap'
    )
    
    # Create the bag recorder node
    bag_recorder_node = Node(
        package='bag_recorder',
        executable='bag_recorder_node.py',
        name='bag_recorder',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'bag_name': LaunchConfiguration('bag_name'),
            'topics': [
                '/navigation_layer/odometry/global',
                '/navigation_layer/odometry/local', 
                '/hardware_layer/diff_cont/odom',
                '/hardware_layer/imu/data_raw',
                '/navsat',
                '/hardware_layer/joint_states'
            ],  # Default topics
            'max_bag_duration': LaunchConfiguration('max_duration'),
            'max_bag_size': LaunchConfiguration('max_size'),
            'compression_mode': LaunchConfiguration('compression_mode'),
            'compression_format': LaunchConfiguration('compression_format'),
            'storage_format': LaunchConfiguration('storage_format'),
        }]
    )
    
    # Log info about the recording
    log_info = LogInfo(
        msg=[
            'Starting sensor data bag recording...\n',
            'Default topics: navigation, hardware layer, and sensor topics\n',
            'Output directory: ', LaunchConfiguration('output_dir'), '\n',
            'Press Ctrl+C to stop recording'
        ]
    )
    
    return LaunchDescription([
        output_dir_arg,
        bag_name_arg,
        topics_arg,
        max_duration_arg,
        max_size_arg,
        compression_mode_arg,
        compression_format_arg,
        storage_format_arg,
        log_info,
        bag_recorder_node,
    ])
