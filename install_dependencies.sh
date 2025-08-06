#!/bin/bash

# ROS2 Localization Workspace - Dependencies Installation Script
# This script installs packages that are not included in ROS2 Humble desktop

set -e  # Exit on any error

echo " Installing ROS2 Localization Workspace Dependencies..."
echo "=================================================="

# Update package list
echo " Updating package list..."
sudo apt update

# ROS2 bag recording tools
echo " Installing ROS2 bag recording tools..."
sudo apt install -y \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rosbag2-compression \
    ros-humble-rosbag2-compression-zstd

# Robot localization (EKF)
echo " Installing robot localization..."
sudo apt install -y ros-humble-robot-localization

# Python dependencies for IMU and other packages
echo " Installing Python dependencies..."
sudo apt install -y \
    python3-serial \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-pip

# Install additional Python packages via pip
echo " Installing additional Python packages..."
pip3 install --user \
    transforms3d \
    pyserial

# Build tools (if not already installed)
echo " Installing build tools..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-vcstool

# USB and serial port access
echo " Installing USB and serial tools..."
sudo apt install -y \
    udev \
    libusb-1.0-0-dev \
    libusb-dev

# Add user to dialout group for serial port access
echo " Adding user to dialout group for serial port access..."
sudo usermod -a -G dialout $USER

echo ""
echo "✅ All dependencies installed successfully!"
echo ""
echo "⚠️  IMPORTANT: You need to log out and log back in (or reboot) for the dialout group changes to take effect."
echo ""
echo "📋 Next steps:"
echo "1. Log out and log back in (for serial port access)"
echo "2. Build the workspace: ./build_gnss_packages.sh"
echo "3. Launch the system: ros2 launch gnss_imu_robot_localization bringup.launch.py"
echo ""
echo "🎒 For data recording, use:"
echo "   ros2 launch gnss_imu_robot_localization bringup.launch.py enable_bag_recording:=true"
echo ""
echo "🔧 If IMU still has issues, use GNSS-only mode:"
echo "   ros2 launch gnss_imu_robot_localization gnss_only_bringup.launch.py"
echo ""
