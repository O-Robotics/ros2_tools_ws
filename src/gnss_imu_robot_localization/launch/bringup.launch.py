from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer, WaitForTopics
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # ---- Launch args ----
    enable_bag_recording_arg = DeclareLaunchArgument(
        'enable_bag_recording',
        default_value='false',
        description='Enable automatic bag recording during localization'
    )

    bag_config_file_arg = DeclareLaunchArgument(
        'bag_config_file',
        default_value='bag_config.yaml',
        description='YAML config file for bag recording (bag_config.yaml, compressed_config.yaml, extended_sensors_config.yaml)'
    )

    # ---- C2D receiver args ----
    c2d_connection_string_arg = DeclareLaunchArgument(
        'c2d_connection_string',
        default_value='',
        description='Azure IoT Hub device connection string for C2D receiver (optional, can be provided via env_file)'
    )
    c2d_env_file_arg = DeclareLaunchArgument(
        'c2d_env_file',
        default_value='/home/dev/ORobotics/secrets/.env',
        description='Path to .env containing IOTHUB_DEVICE_CONNECTION_STRING for C2D receiver'
    )
    c2d_save_dir_arg = DeclareLaunchArgument(
        'c2d_save_dir',
        default_value='/home/dev/ORobotics/localization_ws/src/iot_c2d_receiver/waypoints',
        description='Directory to store waypoint YAMLs for C2D receiver'
    )

    # ---- GNSS (u-blox) params ----
    ublox_params = [
        {'CFG_USBOUTPROT_NMEA': False},
        {'CFG_RATE_MEAS': 10},
        {'CFG_RATE_NAV': 100},
        {'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1},  # For RTK
        {'CFG_MSGOUT_UBX_NAV_STATUS_USB': 1},
        {'CFG_MSGOUT_UBX_NAV_COV_USB': 1},
        {'CFG_MSGOUT_UBX_RXM_RTCM_USB': 1},
        # {'CFG_MSGOUT_UBX_NAV_POSLLH_USB': 1}, # Only lat, lon, alt
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

    # ---- Wait until critical topics are visible, THEN start recording ----
    # Adjust topic names if your drivers publish differently.
    wait_for_sensors = WaitForTopics(
        topics=[
            '/imu/data_raw',  # IMU raw/data topic used by your recorder
            '/fix',           # GNSS NavSatFix
            '/tf',            # TF tree being published (e.g., by robot_state_publisher)
            '/tf_static'      # Optionally add '/tf_static' if you want to enforce it too
        ],
        timeout=20.0  # generous to survive cold GNSS/slow bringup
    )

    # ---- Bag recorder (conditional include) ----
    bag_recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bag_recorder'),
                'launch',
                'bag_record_yaml.launch.py'
            ])
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('bag_config_file')
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_bag_recording'))
    )

    return LaunchDescription([
        # Args
        enable_bag_recording_arg,
        bag_config_file_arg,
        c2d_connection_string_arg,
        c2d_env_file_arg,
        c2d_save_dir_arg,

        # URDF + TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('amr_sweeper_description'),
                    'launch',
                    'rsp.launch.py'
                ])
            )
        ),

        # IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('wit_ros2_imu'),
                    'rviz_and_imu.launch.py'
                ])
            )
        ),

        # GNSS container
        ublox_container,

        # EKF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('gnss_imu_robot_localization'),
                    'launch',
                    'ekf_odm.launch.py'
                ])
            )
        ),

        # C2D Receiver (Azure IoT Hub -> save waypoints)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('iot_c2d_receiver'),
                    'launch',
                    'c2d_receiver.launch.py'
                ])
            ),
            launch_arguments={
                'connection_string': LaunchConfiguration('c2d_connection_string'),
                'env_file': LaunchConfiguration('c2d_env_file'),
                'save_dir': LaunchConfiguration('c2d_save_dir'),
            }.items()
        ),

        # Only after these topics are visible, proceed to the recorder
        wait_for_sensors,

        # Bag recorder (conditional)
        bag_recorder,
    ])
