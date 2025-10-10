from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    connection_string = LaunchConfiguration('connection_string')
    save_dir = LaunchConfiguration('save_dir')
    env_file = LaunchConfiguration('env_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'connection_string',
            default_value='',
            description='Azure IoT Hub device connection string. If empty, uses env IOTHUB_DEVICE_CONNECTION_STRING.'
        ),
        DeclareLaunchArgument(
            'save_dir',
            default_value='/home/dev/ORobotics/localization_ws/src/iot_c2d_receiver/waypoints',
            description='Directory to store waypoint YAMLs (will create dated subfolders).'
        ),
        DeclareLaunchArgument(
            'env_file',
            default_value='',
            description='Optional path to a .env file to load (IOTHUB_DEVICE_CONNECTION_STRING).'
        ),
        Node(
            package='iot_c2d_receiver',
            executable='c2d_receiver',
            name='c2d_receiver',
            output='screen',
            parameters=[{
                'connection_string': connection_string,
                'save_dir': save_dir,
                'env_file': env_file,
            }]
        )
    ])
