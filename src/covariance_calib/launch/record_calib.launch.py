#!/usr/bin/env python3
"""
Launch file for covariance calibration data recording.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    phase_arg = DeclareLaunchArgument(
        'phase',
        default_value='',
        description='Specific phase to record (stationary, slow_spin, slow_straight). Leave empty for --all'
    )
    
    duration_arg = DeclareLaunchArgument(
        'duration',
        default_value='120',
        description='Recording duration in seconds'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='~/calib_data',
        description='Output directory for bag files'
    )
    
    all_phases_arg = DeclareLaunchArgument(
        'all_phases',
        default_value='true',
        description='Record all phases if no specific phase is given'
    )

    # Get launch configurations
    phase = LaunchConfiguration('phase')
    duration = LaunchConfiguration('duration')
    output_dir = LaunchConfiguration('output_dir')
    all_phases = LaunchConfiguration('all_phases')
    
    # Command for recording specific phase
    record_specific_phase = ExecuteProcess(
        condition=UnlessCondition(
            [TextSubstitution(text=''), '==', phase]  # If phase is not empty
        ),
        cmd=[
            'ros2', 'run', 'covariance_calib', 'record_calib',
            '--phase', phase,
            '--duration', duration,
            '--output', output_dir
        ],
        output='screen',
        shell=False
    )
    
    # Command for recording all phases
    record_all_phases = ExecuteProcess(
        condition=IfCondition(
            [TextSubstitution(text=''), '==', phase]  # If phase is empty
        ),
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
            'Starting covariance calibration recording...\n',
            'Phase: ', phase, '\n',
            'Duration: ', duration, ' seconds\n',
            'Output: ', output_dir
        ]
    )

    return LaunchDescription([
        # Arguments
        phase_arg,
        duration_arg,
        output_dir_arg,
        all_phases_arg,
        
        # Actions
        log_info,
        record_specific_phase,
        record_all_phases,
    ])
