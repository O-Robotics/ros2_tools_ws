# Simple EKF test launch file
# Minimal configuration to test basic odom + IMU fusion

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory(
        "nav2_gps_waypoint_follower_demo")
    rl_params_file = os.path.join(
        gps_wpf_dir, "config", "simple_ekf_test.yaml")

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": False}],
                remappings=[("odometry/filtered", "odometry/simple_test")],
            ),
        ]
    )
