
# AMR-Sweeper Robot State Publisher

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # The name of the package and path to xacro file within the package
    package_name = 'amr_sweeper_description'
    package_urdf = 'amr_sweeper.urdf.xacro'


    # Flags to launch as simulation or real robot
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')


    # Use xacro to process the file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'urdf',package_urdf)
    

    # Set Robot State Publisher node parameters
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    rsp_params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}

    # Configure the state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[rsp_params]
    )


    # Run the nodes
    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'use_ros2_control', default_value='true',
            description='Use ros2_control if true'),

        node_robot_state_publisher

    ])
