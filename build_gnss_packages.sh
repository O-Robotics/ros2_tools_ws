#!/bin/bash

# ROS2 GNSS/DGNSS Packages Build Script
# This script builds the UBlox GNSS/DGNSS packages in the correct dependency order
# to ensure reproducible builds across different devices.

set -e  # Exit on any error

echo "=========================================="
echo "ROS2 GNSS/DGNSS Packages Build Script"
echo "=========================================="

# Check if we're in a ROS2 workspace
if [ ! -f "src/CMakeLists.txt" ] && [ ! -d "src" ]; then
    echo "Error: This script must be run from the root of a ROS2 workspace"
    exit 1
fi

# Source ROS2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "Sourcing ROS2 Humble environment..."
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    echo "Sourcing ROS2 Foxy environment..."
    source /opt/ros/foxy/setup.bash
else
    echo "Warning: No ROS2 environment found. Make sure ROS2 is installed and sourced."
fi

# Clean previous builds (optional - uncomment if needed)
# echo "Cleaning previous builds..."
# rm -rf build/ install/ log/

echo "Building all workspace packages in dependency order..."
echo "Building IMU and GNSS packages in parallel..."
colcon build \
  --packages-select rtcm_msgs \
                    ublox_ubx_interfaces \
                    ublox_ubx_msgs \
                    ublox_dgnss_node \
                    ublox_nav_sat_fix_hp_node \
                    ntrip_client_node \
                    ublox_dgnss \
                    wit_ros2_imu \
  --symlink-install \
  -j$(nproc)
  
#Build other localization packages (independent)
echo ""
echo "Step 1/4: Building core localization packages..."
colcon build --packages-select amr_sweeper_description bag_recorder gnss_imu_robot_localization imu_offset_calibration

#Build main DGNSS package and working Nav2 demos
echo ""
echo "Step 3/4: Building DGNSS package and Nav2 demos..."
colcon build --packages-select nav2_costmap_filters_demo nav2_gps_waypoint_follower_demo


echo ""
echo "=========================================="
echo "✅ All workspace packages built successfully!"
echo "=========================================="
echo ""
echo "Sourcing the workspace automatically..."
source install/setup.bash
echo "✅ Workspace sourced successfully!"
echo ""
echo "Note: In new terminals, remember to run:"
echo "  source install/setup.bash"
echo ""
echo "Built packages:"
echo ""
echo "Core Localization:"
echo "  - amr_sweeper_description"
echo "  - bag_recorder"
echo "  - gnss_imu_robot_localization"
echo "  - imu_offset_calibration"
echo ""
echo "GNSS/DGNSS Stack:"
echo "  - rtcm_msgs"
echo "  - ublox_ubx_interfaces"
echo "  - ublox_ubx_msgs"
echo "  - ublox_dgnss_node"
echo "  - ublox_nav_sat_fix_hp_node"
echo "  - ntrip_client_node"
echo "  - ublox_dgnss"
echo ""
echo "IMU:"
echo "  - wit_ros2_imu"
echo ""
echo "Navigation2 Demos:"
echo "  - nav2_costmap_filters_demo"
echo "  - nav2_gps_waypoint_follower_demo"
