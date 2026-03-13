# Tools Workspace

ROS2 packages like ros2 bag recorder, sensor covariance calculator, etc


## Bag recorder

### Full configuration example
```bash
ros2 launch bag_recorder bag_record.launch.py \
  output_dir:="~/robot8_aarhus_indoor_031326" \
  bag_name:="my_recording" \
  max_duration:="300" \
  max_size:="1000" \
  compression_mode:="none" \
  storage_format:="mcap"
```

### Parameters
- output_dir: Directory to save bag files (default: ~/ros2_bags)
- bag_name: Custom bag name (default: auto-generated with timestamp)
- max_duration: Maximum recording time in seconds (0 = unlimited)
- max_size: Maximum file size in MB (0 = unlimited)
- compression_mode: `none`, `file`, or `message` (default: none)
- compression_format: `none`, `zstd` or `lz4` (default: zstd)
- storage_format: `sqlite3` or `mcap` (default: sqlite3)
- Stop Recording
- Press `Ctrl+C` to stop recording gracefully.

### Tips

Recomendations: 
- bag_name: ~/robot8_aarhus_outdoor_031326: With which robot, where, indoor or outdoor and date like day-month-year
- compression_format: `none`: You need to decompress before using covariance calculator, so better not compress if it's not big
- storage_format: `mcap`: It can be replayed in Foxglove directly

### topic it records for now
```
/navigation_layer/odometry/global
/navigation_layer/odometry/local
/hardware_layer/diff_cont/odom
/hardware_layer/imu/data_raw
/navsat
/hardware_layer/joint_states
```
### how to change the topic

#### one time usage
```
ros2 launch bag_recorder bag_record.launch.py \
  topics:='["/my_topic1", "/my_topic2", "/camera/image_raw"]'
```

#### change the file
You will need to change `\bag_recorder.py` and `\launch\bag_record.launch.py`
Or feel free to have your own launch file for specific topics







## Outdated

### Option 1: Build All Packages
```bash
# Build entire workspace
colcon build
source install/setup.bash
```

### Option 2: Build GNSS/DGNSS Packages Only (Recommended)
```bash
# Use the automated build script for reproducible builds
./build_gnss_packages.sh
source install/setup.bash
```

### Option 3: Manual Sequential Build
```bash
# Step 1: Build core interfaces and nodes
colcon build --packages-select ublox_ubx_interfaces ublox_ubx_msgs ublox_dgnss_node ublox_nav_sat_fix_hp_node ntrip_client_node

# Step 2: Build main DGNSS package
colcon build --packages-select ublox_dgnss

# Step 3: Build IMU package
colcon build --packages-select wit_ros2_imu

source install/setup.bash
```

### Launch System
```bash
# Launch localization
ros2 launch gnss_imu_robot_localization bringup.launch.py

# Launch with data recording
ros2 launch gnss_imu_robot_localization bringup.launch.py \
    enable_bag_recording:=true \
    bag_config_file:="compressed_config.yaml"
```

## Packages

### Core Localization
- **gnss_imu_robot_localization** - Main localization system using EKF
- **bag_recorder** - Sensor data recording utility
- **imu_offset_calibration** - IMU calibration tools
- **amr_sweeper_description** - Robot description files

### GNSS/DGNSS Stack
- **ublox_ubx_interfaces** - UBlox UBX message interfaces
- **ublox_ubx_msgs** - UBlox UBX message definitions
- **ublox_dgnss_node** - Main DGNSS receiver node
- **ublox_nav_sat_fix_hp_node** - High-precision navigation satellite fix
- **ntrip_client_node** - NTRIP client for RTK corrections
- **ublox_dgnss** - Main UBlox DGNSS package
- **rtcm_msgs** - RTCM message definitions

### IMU
- **wit_ros2_imu** - WIT IMU driver

### Navigation2 Demos (Optional)
- **nav2_costmap_filters_demo** - Costmap filters demonstration
- **nav2_gps_waypoint_follower_demo** - GPS waypoint following
- **nav2_gradient_costmap_plugin** - Custom costmap plugin

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

## Reproducible Build Process

For new device setup, the GNSS/DGNSS packages must be built in a specific order due to dependencies:

### Automated Build (Recommended)
Use the provided build script:
```bash
./build_gnss_packages.sh
```

### Manual Build Order
If building manually, follow this exact sequence:

1. **First batch** (can be built in parallel):
   ```bash
   colcon build --packages-select ublox_ubx_interfaces ublox_ubx_msgs ublox_dgnss_node ublox_nav_sat_fix_hp_node ntrip_client_node
   ```

2. **Second batch** (depends on first batch):
   ```bash
   colcon build --packages-select ublox_dgnss
   ```

3. **IMU package** (independent):
   ```bash
   colcon build --packages-select wit_ros2_imu
   ```

### Why This Order Matters
- `ublox_ubx_interfaces` and `ublox_ubx_msgs` provide message definitions needed by other packages
- `ublox_dgnss_node`, `ublox_nav_sat_fix_hp_node`, and `ntrip_client_node` depend on the message definitions
- `ublox_dgnss` is a meta-package that depends on all the above components
- `wit_ros2_imu` is independent and can be built separately

### Troubleshooting Build Issues
- If packages fail to build, clean the workspace: `rm -rf build/ install/ log/`
- Ensure ROS2 environment is sourced: `source /opt/ros/humble/setup.bash`
- Check that all dependencies are installed (see Additional Dependencies section)
- Use the automated build script to ensure correct build order

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


