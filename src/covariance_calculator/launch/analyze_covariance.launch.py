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
        default_value='config/analysis_config.yaml',
        description='Path to analysis configuration file'
    )
    
    data_directory_arg = DeclareLaunchArgument(
        'data_directory',
        default_value='data/my_bags_outdoor_25',
        description='Directory containing bag files to analyze'
    )
    
    output_directory_arg = DeclareLaunchArgument(
        'output_directory',
        default_value='results',
        description='Directory to save analysis results'
    )
    
    # Create the covariance calculator node
    covariance_calculator_node = Node(
        package='covariance_calculator',
        executable='covariance_calculator_node',
        name='covariance_calculator',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'data_directory': LaunchConfiguration('data_directory'),
            'output_directory': LaunchConfiguration('output_directory'),
        }]
    )
    
    # Log info about the analysis
    log_info = LogInfo(
        msg=[
            'Starting covariance analysis...\n',
            'Data directory: ', LaunchConfiguration('data_directory'), '\n',
            'Output directory: ', LaunchConfiguration('output_directory'), '\n',
            'This will analyze IMU and Odometry covariance from bag data'
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        data_directory_arg,
        output_directory_arg,
        log_info,
        covariance_calculator_node,
    ])
