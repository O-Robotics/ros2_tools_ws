#!/usr/bin/env python3
"""
Launch file for covariance calculation and analysis.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Declare launch arguments
    bags_root_arg = DeclareLaunchArgument(
        'bags_root',
        description='Root directory containing calibration bags (required)'
    )
    
    run_phases_arg = DeclareLaunchArgument(
        'run_phases',
        default_value='',
        description='Specific phases to analyze (space-separated: stationary slow_spin slow_straight). Leave empty for all'
    )
    
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/hardware_layer/imu/data_raw',
        description='IMU topic name'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/hardware_layer/diff_cont/odom',
        description='Odometry topic name'
    )
    
    out_dir_arg = DeclareLaunchArgument(
        'out_dir',
        default_value='',
        description='Output directory (default: auto-generated based on input)'
    )
    
    segment_mode_arg = DeclareLaunchArgument(
        'segment_mode',
        default_value='auto',
        choices=['auto', 'trim_only', 'threshold_only', 'raw'],
        description='Data segmentation strategy'
    )
    
    min_samples_arg = DeclareLaunchArgument(
        'min_samples',
        default_value='',
        description='Minimum samples required (overrides config)'
    )
    
    wz_min_arg = DeclareLaunchArgument(
        'wz_min',
        default_value='',
        description='Minimum angular velocity for slow_spin (rad/s)'
    )
    
    vx_min_arg = DeclareLaunchArgument(
        'vx_min',
        default_value='',
        description='Minimum linear velocity for slow_straight (m/s)'
    )

    # Get launch configurations
    bags_root = LaunchConfiguration('bags_root')
    run_phases = LaunchConfiguration('run_phases')
    imu_topic = LaunchConfiguration('imu_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    out_dir = LaunchConfiguration('out_dir')
    segment_mode = LaunchConfiguration('segment_mode')
    min_samples = LaunchConfiguration('min_samples')
    wz_min = LaunchConfiguration('wz_min')
    vx_min = LaunchConfiguration('vx_min')
    
    # Build command arguments dynamically
    cmd_args = [
        'ros2', 'run', 'covariance_calib', 'calculate_cov',
        '--bags_root', bags_root,
        '--imu_topic', imu_topic,
        '--odom_topic', odom_topic,
        '--segment_mode', segment_mode
    ]
    
    # Command for calculation
    calculate_covariance = ExecuteProcess(
        cmd=cmd_args,
        additional_env={'PYTHONUNBUFFERED': '1'},
        output='screen',
        shell=False
    )
    
    # Log info
    log_info = LogInfo(
        msg=[
            'Starting covariance calculation...\n',
            'Input bags: ', bags_root, '\n',
            'IMU topic: ', imu_topic, '\n',
            'Odom topic: ', odom_topic, '\n',
            'Segment mode: ', segment_mode
        ]
    )

    return LaunchDescription([
        # Arguments
        bags_root_arg,
        run_phases_arg,
        imu_topic_arg,
        odom_topic_arg,
        out_dir_arg,
        segment_mode_arg,
        min_samples_arg,
        wz_min_arg,
        vx_min_arg,
        
        # Actions
        log_info,
        calculate_covariance,
    ])
