from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import WaitForTopics, ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # URDF + TF
    amr_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('amr_sweeper_description'),
                'launch',
                'rsp.launch.py'
            ])
        )
    )

    # IMU driver + RViz helper
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('wit_ros2_imu'),
                'launch',
                'rviz_and_imu.launch.py'
            ])
        )
    )

    # u-blox GNSS container (same as bringup)
    ublox_params = [
        {'CFG_USBOUTPROT_NMEA': False},
        {'CFG_RATE_MEAS': 10},
        {'CFG_RATE_NAV': 100},
        {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1},
        {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 1},
        {'CFG_MSGOUT_UBX_NAV_COV_USB': 1},
        {'CFG_MSGOUT_UBX_RXM_RTCM_USB': 1},
    ]

    ublox_container = ComposableNodeContainer(
        name='ublox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='ublox_dgnss_node',
                plugin='ublox_dgnss::UbloxDGNSSNode',
                name='ublox_dgnss',
                parameters=ublox_params
            ),
            ComposableNode(
                package='ublox_nav_sat_fix_hp_node',
                plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
                name='ublox_nav_sat_fix_hp'
            ),
        ]
    )

    # Gate: wait until critical topics are available before starting consumers
    wait_for_sensors = WaitForTopics(
        topics=['/imu/data_raw', '/fix', '/tf', '/tf_static'],
        timeout=20.0
    )

    # Robot Localization chain: EKF (local) -> navsat_transform -> EKF (map)
    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odm_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('gnss_imu_robot_localization'),
                'config',
                'ekf_odm.yaml'
            ])
        ],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered/local'),
            # IMU topic already set to /imu/data_raw in YAML; no remap needed
        ]
    )

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('gnss_imu_robot_localization'),
                'config',
                'navsat_transform.yaml'
            ])
        ],
        remappings=[
            ('/gps/fix', '/fix'),
            ('/odometry/filtered', '/odometry/filtered/local'),
        ]
    )

    ekf_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_map_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('gnss_imu_robot_localization'),
                'config',
                'ekf_map.yaml'
            ])
        ],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered/global'),
        ]
    )

    return LaunchDescription([
        amr_rsp,
        imu_launch,
        ublox_container,
        wait_for_sensors,
        ekf_local,
        navsat_transform,
        ekf_map,
    ])
