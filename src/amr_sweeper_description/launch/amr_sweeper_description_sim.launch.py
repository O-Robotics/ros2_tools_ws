

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description():

    # Specify the name of the package
    package_name = 'amr_sweeper_description'
    

    # Launch AMR-Sweeper Robot State Publisher with sim_time=true and ros2_control=false
    sim_launch = IncludeLaunchDescription(           
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py')]), 
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )


    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )


    # Open RVIZ2
    rviz2_params = os.path.join(get_package_share_directory(package_name),'config','rviz2_config.rviz')
    rviz2 = Node(                                
        package='rviz2',
        namespace='', 
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_params]   
    )


    #Path to world file[spawner_joint_broad]: waiting for service /controller_manager/list_controllers to become available...
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )


    # Include Gazebo launch file
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args':['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
   )


    # Spawn Sweepe in Gazebo
    spawn_amrsweeper_sim = Node(                                #note AMR-Sweeper name without "-" due to python
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'amr_sweeper_description', '-z', '0.13'],
        output='screen'
    )



    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )


    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )


    # Run all
    return LaunchDescription([
        sim_launch,                      
        twist_mux,
        rviz2,
        world_arg,
        gz_sim,
        spawn_amrsweeper_sim,                       #note AMR-Sweeper name without "-"
        ros_gz_bridge,
        ros_gz_image_bridge
    ])
