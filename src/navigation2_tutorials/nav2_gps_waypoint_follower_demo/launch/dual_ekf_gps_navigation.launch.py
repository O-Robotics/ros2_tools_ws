# Complete GPS Navigation Launch File
# Dual EKF system with navsat_transform for GPS coordinate conversion

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os

def generate_launch_description():
    gps_wpf_dir = get_package_share_directory("nav2_gps_waypoint_follower_demo")
    rl_params_file = os.path.join(gps_wpf_dir, "config", "dual_ekf_gps_fusion.yaml")

    return LaunchDescription([
        # Local EKF Node (odom frame) - fuses odom + IMU
        launch_ros.actions.Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_odom",
            output="screen",
            parameters=[rl_params_file, {"use_sim_time": False}],
            remappings=[("odometry/filtered", "odometry/local")],
        ),
        
        # Global EKF Node (map frame) - fuses local EKF + GPS
        launch_ros.actions.Node(
            package="robot_localization",
            executable="ekf_node", 
            name="ekf_filter_node_map",
            output="screen",
            parameters=[rl_params_file, {"use_sim_time": False}],
            remappings=[("odometry/filtered", "odometry/global")],
        ),
        
        # Navsat Transform Node - converts GPS to local coordinates
        launch_ros.actions.Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[rl_params_file, {"use_sim_time": False}],
            remappings=[
                ("imu/data", "imu/data_raw"),
                ("gps/fix", "fix"),
                ("odometry/filtered", "odometry/local"),
                ("odometry/gps", "odometry/gps")
            ],
        ),
    ])
