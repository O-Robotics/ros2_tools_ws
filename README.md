# Localization Workspace

ROS2 workspace for autonomous mobile robot localization using GNSS and IMU sensor fusion.

## Quick Start

```bash
# Build workspace
cd /home/dev/ORobotics/localization_ws
colcon build
source install/setup.bash

# Launch localization
ros2 launch gnss_imu_robot_localization bringup.launch.py

# Launch with data recording
ros2 launch gnss_imu_robot_localization bringup.launch.py \
    enable_bag_recording:=true \
    bag_config_file:="compressed_config.yaml"
```

## Packages

- **gnss_imu_robot_localization** - Main localization system
- **bag_recorder** - Sensor data recording
- **wit_ros2_imu** - IMU driver
- **ublox_dgnss*** - GNSS drivers
- **amr_sweeper_description** - Robot description

## Additional Dependencies

These packages need to be installed separately (not included in ROS2 Humble desktop):

```bash
# ROS2 bag recording tools
sudo apt install -y ros-humble-rosbag2-storage-default-plugins
sudo apt install -y ros-humble-rosbag2-storage-sqlite3
sudo apt install -y ros-humble-rosbag2-compression
sudo apt install -y ros-humble-rosbag2-compression-zstd

# Robot localization (EKF)
sudo apt install -y ros-humble-robot-localization

# Serial communication for IMU
sudo apt install -y python3-serial

# Build tools (if not already installed)
sudo apt install -y python3-colcon-common-extensions
```

## Build Commands

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select package_name

# Clean specific package
rm -rf build/package_name install/package_name

# Use rebuild script
./rebuild.sh                    # Build all
./rebuild.sh package_name       # Build specific
./rebuild.sh --clean package    # Clean and build
./rebuild.sh --help             # Show options
```

## Development

```bash
# Selective building (recommended)
colcon build --packages-select gnss_imu_robot_localization
source install/setup.bash

# Clean specific packages only (avoid rm -rf build/ install/ log/)
rm -rf build/package_name install/package_name
```

## Hardware

- ublox F9P GNSS receiver
- WIT motion sensor IMU
- USB connections

---

**Status**: Production Ready
