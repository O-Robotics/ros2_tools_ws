from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import WaitForTopics


def generate_launch_description():
    # Launch args to optionally start downstream processors
    start_navsat_arg = DeclareLaunchArgument(
        'start_navsat', default_value='false',
        description='Start navsat_transform after sensors are up')

    start_ekf_arg = DeclareLaunchArgument(
        'start_ekf', default_value='false',
        description='Start EKF local filter after sensors are up')

    amr_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('amr_sweeper_description'),
                'launch',
                'rsp.launch.py'
            ])
        )
    )

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wit_ros2_imu'),
                'launch',
                'rviz_and_imu.launch.py'
            ])
        )
    )

    ublox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ublox_dgnss'),
                'launch',
                'ublox_rover_hpposllh_navsatfix.launch.py'
            ])
        )
    )

    # Gate: wait until critical topics are available before starting consumers
    wait_for_sensors = WaitForTopics(
        topics=['/imu/data_raw', '/fix', '/tf'],
        timeout=20.0
    )

    # Optional downstream: navsat_transform
    navsat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gnss_imu_robot_localization'),
                'launch',
                'navsat_transform.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('start_navsat'))
    )

    # Optional downstream: EKF local filter
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gnss_imu_robot_localization'),
                'launch',
                'ekf_odm.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('start_ekf'))
    )

    return LaunchDescription([
        start_navsat_arg,
        start_ekf_arg,
        amr_rsp,
        imu_launch,
        ublox_launch,
        wait_for_sensors,
        navsat_launch,
        ekf_launch,
    ])
