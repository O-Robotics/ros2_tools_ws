# Covariance Calculator

A ROS2 package for calculating IMU and Odometry measurement noise covariance matrices from bag data. Designed to help tune sensor parameters for the `robot_localization` package.

## Features

- **IMU Analysis**: Calculates yaw and yaw_rate covariance from sensor_msgs/msg/Imu data
- **Odometry Analysis**: Calculates velocity_x and velocity_y covariance from nav_msgs/msg/Odometry data
- **Motion State Detection**: Automatically detects stationary periods using joint_states wheel velocities
- **Outlier Filtering**: Statistical outlier removal for robust covariance estimation
- **Robot Localization Integration**: Generates ready-to-use configuration for robot_localization package
- **Batch Processing**: Analyze multiple datasets in one run
- **Comprehensive Documentation**: Mathematical formulas and implementation details

## Quick Start

### Prerequisites

Ensure you have collected bag data with three motion types:
- **Stationary**: Robot at rest
- **Slow Spin**: In-place rotation
- **Straight Line**: Linear motion

Required topics:
- `/hardware_layer/imu/data_raw` (sensor_msgs/msg/Imu)
- `/hardware_layer/diff_cont/odom` (nav_msgs/msg/Odometry)
- `/hardware_layer/joint_states` (sensor_msgs/msg/JointState)

### Installation

1. Build the package:
```bash
cd /path/to/your/ros2_ws
colcon build --packages-select covariance_calculator
source install/setup.bash
```

### Basic Usage

1. **Place your bag data** in the package directory:
```
covariance_calculator/
└── data/
    └── my_bags_outdoor_25/
        ├── stationary_recording/
        ├── slow_spin_recording/
        └── slow_straight_recording/
```

2. **Run the analysis**:
```bash
ros2 launch covariance_calculator analyze_covariance.launch.py
```

3. **Check results** in the `results/` directory:
   - `imu_covariance_results.txt` - Detailed IMU analysis
   - `odom_covariance_results.txt` - Detailed Odometry analysis
   - `robot_localization_config.yaml` - Ready-to-use configuration
   - `covariance_analysis_summary.txt` - Summary report

### Custom Configuration

Edit `config/analysis_config.yaml` to customize:

```yaml
covariance_calculator:
  ros__parameters:
    # Data paths
    data_directory: "data/my_bags_outdoor_25"
    stationary_bag: "stationary_recording"
    spin_bag: "slow_spin_recording"
    straight_bag: "slow_straight_recording"
    
    # Analysis parameters
    stationary_detection:
      wheel_velocity_threshold: 0.01  # rad/s
      min_stationary_duration: 2.0   # seconds
    
    imu_analysis:
      filter_outliers: true
      outlier_std_threshold: 3.0
    
    odom_analysis:
      filter_outliers: true
      outlier_std_threshold: 3.0
```

## Usage Methods

### Method 1: Launch File (Recommended)

```bash
# Basic usage
ros2 launch covariance_calculator analyze_covariance.launch.py

# Custom data directory
ros2 launch covariance_calculator analyze_covariance.launch.py \
    data_directory:="path/to/your/bags" \
    output_directory:="custom_results"
```

### Method 2: Direct Node Execution

```bash
ros2 run covariance_calculator covariance_calculator_node \
    --ros-args \
    -p data_directory:="data/my_bags_outdoor_25" \
    -p output_directory:="results"
```

### Method 3: Batch Analysis

```bash
# Analyze multiple datasets
ros2 run covariance_calculator batch_analyze \
    --data-dir /path/to/multiple/datasets \
    --output-dir batch_results
```

## Output Files

### IMU Covariance Results
- Yaw variance (rad²)
- Yaw rate variance ((rad/s)²)
- Cross-covariance between yaw and yaw_rate
- Statistical analysis for each motion type

### Odometry Covariance Results
- Velocity X variance ((m/s)²)
- Velocity Y variance ((m/s)²)
- Cross-covariance between velocity components
- Motion-specific insights and warnings

### Robot Localization Configuration
Ready-to-use YAML configuration:

```yaml
imu0: /hardware_layer/imu/data_raw
imu0_config: [false, false, false,
              false, false, true,   # yaw
              false, false, false,
              false, false, true,   # yaw_rate
              false, false, false]
imu0_covariance: [0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, σ²_yaw]

odom0: /hardware_layer/diff_cont/odom
odom0_config: [false, false, false,
               false, false, false,
               true,  true,  false,  # vx, vy
               false, false, false,
               false, false, false]
```

## Mathematical Background

The package implements classical sample covariance calculation:

**Sample Variance:**
```
σ² = (1/(N-1)) * Σᵢ₌₁ᴺ (xᵢ - x̄)²
```

**Sample Covariance:**
```
Cov(x,y) = (1/(N-1)) * Σᵢ₌₁ᴺ (xᵢ - x̄)(yᵢ - ȳ)
```

See `docs/covariance_formulas.md` for complete mathematical documentation.

## Motion State Analysis

### Stationary State
- **IMU**: Measures sensor noise and bias drift
- **Odometry**: Measures encoder noise and wheel slip

### Spinning State
- **IMU**: Analyzes gyroscope accuracy during rotation
- **Odometry**: Measures velocity noise during in-place rotation

### Straight Line State
- **IMU**: Measures yaw stability during translation
- **Odometry**: Analyzes forward velocity accuracy and lateral drift

## Integration with Robot Localization

Copy the generated configuration to your robot_localization launch file:

```python
# In your robot_localization launch file
ekf_config = os.path.join(
    get_package_share_directory('your_package'),
    'config',
    'robot_localization_config.yaml'  # Generated by covariance_calculator
)

ekf_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    parameters=[ekf_config]
)
```

## Troubleshooting

### Common Issues

**No stationary periods detected:**
- Check `wheel_velocity_threshold` in config
- Ensure joint_states contains wheel velocities
- Verify robot was actually stationary during recording

**Insufficient data warnings:**
- Increase recording duration
- Check topic names match configuration
- Verify bag files contain expected topics

**High covariance values:**
- Check sensor calibration
- Verify mechanical connections
- Consider environmental factors (vibration, temperature)

### Debug Mode

```bash
ros2 launch covariance_calculator analyze_covariance.launch.py \
    --ros-args --log-level DEBUG
```

## Package Structure

```
covariance_calculator/
├── covariance_calculator/           # Python package
│   ├── data_analyzer.py            # Data loading and preprocessing
│   ├── imu_covariance_analyzer.py  # IMU covariance calculation
│   ├── odom_covariance_analyzer.py # Odometry covariance calculation
│   ├── covariance_calculator_node.py # Main node
│   ├── batch_analyze.py            # Batch processing script
│   └── utils.py                    # Utility functions
├── launch/                         # Launch files
├── config/                         # Configuration files
├── docs/                          # Documentation
├── data/                          # Bag data (user-provided)
└── results/                       # Analysis outputs
```

## Future Extensions

The package is designed for extensibility:

- **Allan Variance Analysis**: For advanced noise characterization
- **Spectral Analysis**: Frequency-domain noise analysis
- **Additional Sensors**: Camera, LiDAR, wheel odometry
- **Real-time Analysis**: Online covariance estimation

## Dependencies

- rclpy
- sensor_msgs
- nav_msgs
- geometry_msgs
- rosbag2_py
- numpy
- pyyaml

## License

MIT

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Submit a pull request

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review `docs/covariance_formulas.md`
3. Open an issue with bag data details and error logs
