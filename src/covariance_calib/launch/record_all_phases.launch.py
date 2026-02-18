#!/usr/bin/env python3
"""
Launch file for recording all calibration phases sequentially.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    duration_arg = DeclareLaunchArgument(
        'duration',
        default_value='120',
        description='Base recording duration in seconds (phases may override with recommended durations)'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='~/calib_data',
        description='Output directory for bag files'
    )

    # Get launch configurations
    duration = LaunchConfiguration('duration')
    output_dir = LaunchConfiguration('output_dir')
    
    # Command for recording all phases
    record_all_phases = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'covariance_calib', 'record_calib',
            '--all',
            '--duration', duration,
            '--output', output_dir
        ],
        output='screen',
        shell=False
    )
    
    # Log info
    log_info = LogInfo(
        msg=[
            'Starting sequential recording of all calibration phases...\n',
            'Base duration: ', duration, ' seconds\n',
            'Output: ', output_dir, '\n',
            'Phases: stationary -> slow_spin -> slow_straight'
        ]
    )

    return LaunchDescription([
        # Arguments
        duration_arg,
        output_dir_arg,
        
        # Actions
        log_info,
        record_all_phases,
    ])
