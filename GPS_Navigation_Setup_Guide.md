# GPS Navigation System Setup Guide

Complete guide for setting up dual EKF GPS navigation system with navsat_transform.

## System Architecture

```
GPS (/fix) ──┐
             ├─→ navsat_transform ─→ /odometry/gps ──┐
IMU (/imu/data_raw) ──┤                               │
             └─→ ekf_local ─→ /odometry/local ────────┼─→ ekf_global ─→ /odometry/global
ODOM (/odom) ────────┘                                │
                                                      │
                                               (map ← odom TF)
```

## Components

1. **Local EKF** (`ekf_filter_node_odom`): Fuses odom + IMU → `/odometry/local`
2. **Navsat Transform** (`navsat_transform_node`): Converts GPS → `/odometry/gps`  
3. **Global EKF** (`ekf_filter_node_map`): Fuses local + GPS → `/odometry/global`

## Build and Setup

```bash
# 1. Build the package
cd /home/dev/ORobotics/localization_ws
colcon build --packages-select nav2_gps_waypoint_follower_demo

# 2. Source the workspace
source install/setup.bash

# 3. Ensure all sensor nodes are running
ros2 topic list | grep -E "(fix|imu|odom)"
# Should show: /fix, /imu/data_raw, /odom
```

## Launch Commands

### Step 1: Start Sensor Publishers
```bash
# Terminal 1: Start simple odom publisher
ros2 run nav2_gps_waypoint_follower_demo simple_odom_publisher

# Terminal 2: Ensure GPS and IMU are publishing
ros2 topic hz /fix
ros2 topic hz /imu/data_raw
```

### Step 2: Launch Complete GPS Navigation
```bash
# Terminal 3: Launch dual EKF + navsat_transform
ros2 launch nav2_gps_waypoint_follower_demo dual_ekf_gps_navigation.launch.py
```

## Verification Steps

### 1. Check Topic Output
```bash
# Local EKF output (odom + IMU fusion)
ros2 topic echo /odometry/local --once

# GPS coordinate conversion
ros2 topic echo /odometry/gps --once

# Global fusion result
ros2 topic echo /odometry/global --once
```

### 2. Monitor Data Flow
```bash
# Check topic frequencies
ros2 topic hz /odometry/local
ros2 topic hz /odometry/gps  
ros2 topic hz /odometry/global

# Monitor diagnostics
ros2 topic echo /diagnostics | grep -A5 "ekf"
```

### 3. Verify TF Tree
```bash
# Generate TF tree visualization
ros2 run tf2_tools view_frames

# Check specific transforms
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

## Expected Behavior

### Startup Sequence
1. **Local EKF starts**: `/odometry/local` begins publishing
2. **Navsat waits for datum**: GPS coordinate origin establishment
3. **GPS conversion starts**: `/odometry/gps` begins publishing  
4. **Global fusion**: `/odometry/global` provides final result

### Normal Operation
- **Local EKF**: Smooth, high-frequency position estimates
- **GPS conversion**: Periodic corrections based on GPS updates
- **Global EKF**: Drift-corrected global positioning

## Troubleshooting

### No GPS Conversion Output
```bash
# Check navsat_transform status
ros2 topic echo /diagnostics | grep navsat

# Verify GPS data quality
ros2 topic echo /fix --once
# Look for: status.status >= 0, position_covariance reasonable
```

### EKF Rejection Issues
```bash
# Monitor rejection diagnostics
ros2 topic echo /diagnostics | grep -A5 "rejection"

# Check sensor data quality
ros2 topic echo /odom --once
ros2 topic echo /imu/data_raw --once
```

### TF Transform Issues
```bash
# Check for TF warnings
ros2 run tf2_ros tf2_monitor

# Verify frame relationships
ros2 run rqt_tf_tree rqt_tf_tree
```

## Configuration Tuning

### GPS Rejection Thresholds
```yaml
# In dual_ekf_gps_fusion.yaml
odom1_pose_rejection_threshold: 15.0  # Increase if GPS rejected too often
```

### Process Noise Tuning
```yaml
# Higher values = trust sensors less, allow more model uncertainty
process_noise_covariance: [0.1, 0.0, ...]  # Position noise
```

### Navsat Transform Parameters
```yaml
# GPS coordinate conversion settings
delay: 3.0                    # Wait time for initial datum
use_odometry_yaw: true       # Use EKF yaw instead of GPS heading
wait_for_datum: false        # Don't wait indefinitely for GPS
```

## Performance Monitoring

### Key Metrics
- **Local EKF frequency**: Should maintain ~2Hz
- **GPS update rate**: Typically 1-10Hz depending on receiver
- **Global EKF convergence**: Position should stabilize within 30s
- **Rejection rate**: <5% for healthy operation

### Diagnostic Commands
```bash
# Real-time monitoring
watch -n 1 'ros2 topic hz /odometry/global'

# Log analysis
ros2 bag record /odometry/local /odometry/gps /odometry/global /diagnostics
```

## Integration with Navigation

Once GPS navigation is working, integrate with Nav2:

```bash
# Launch complete navigation stack
ros2 launch nav2_gps_waypoint_follower_demo dual_ekf_navsat_custom.launch.py

# Send GPS waypoints
ros2 service call /follow_gps_waypoints nav2_msgs/srv/FollowWaypoints "..."
```

## Common Issues and Solutions

| Issue | Symptom | Solution |
|-------|---------|----------|
| No GPS conversion | `/odometry/gps` not publishing | Check GPS fix quality, increase delay |
| EKF divergence | Position jumps/oscillates | Tune rejection thresholds, check sensor sync |
| TF warnings | Transform lookup failures | Verify frame names, check node startup order |
| Poor GPS accuracy | Large position errors | Check multipath, satellite visibility |

This system provides robust GPS-based navigation suitable for outdoor autonomous operations.
