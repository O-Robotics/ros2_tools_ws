# Localization Calibration Tool

A standalone tool for calibrating sensor covariances for localization systems. This tool records sensor data in different motion phases and calculates optimal covariance values for IMU and odometry sensors.

## Directory Structure

```
tools/
  localization_calib/
    README.md                    # This file
    record/
      record_calib.py           # Recording script
      topics.yaml               # Topic configuration
    calculate/
      calculate_cov.py          # Covariance calculation script
      config.yaml               # Calculation parameters
    data/
      bags/
        2026-02-18/             # Date-based organization
          calib_stationary/
          calib_slow_spin/
          calib_slow_straight/
      results/
        2026-02-18/
          imu/
            stationary.json
            slow_spin.json
            slow_straight.json
            summary.json
          odom/
            stationary.json
            slow_spin.json
            slow_straight.json
            summary.json
          report.md
```

## Usage

### 1. Recording Calibration Data

```bash
# Record all phases (stationary, slow_spin, slow_straight)
python record/record_calib.py --all --duration 120

# Record specific phase
python record/record_calib.py --phase stationary --duration 60
python record/record_calib.py --phase slow_spin --duration 90
python record/record_calib.py --phase slow_straight --duration 90

# Custom output directory
python record/record_calib.py --all --output data/bags/2026-02-18/
```

### 2. Calculating Covariances

#### Basic Analysis
```bash
# Standard analysis with auto segmentation
python calculate/calculate_cov.py --bags_root data/bags/2026-02-18/

# Analyze specific phases only
python calculate/calculate_cov.py --bags_root data/bags/2026-02-18/ --run stationary slow_spin
```

#### Segmentation Control
```bash
# Custom thresholds for different chassis
python calculate/calculate_cov.py --bags_root data/bags/2026-02-18/ \
  --segment_mode threshold_only --wz_min 0.1 --vx_min 0.05

# Raw data analysis without filtering
python calculate/calculate_cov.py --bags_root data/bags/2026-02-18/ \
  --segment_mode raw

# Only time-based trimming
python calculate/calculate_cov.py --bags_root data/bags/2026-02-18/ \
  --segment_mode trim_only
```

#### Advanced Options
```bash
# Custom topics and output directory
python calculate/calculate_cov.py \
  --bags_root data/bags/2026-02-18/ \
  --imu_topic /hardware_layer/imu/data_raw \
  --odom_topic /hardware_layer/diff_cont/odom \
  --out_dir data/results/2026-02-18/ \
  --segment_mode auto --min_samples 200
```

## Calibration Phases

### Stationary
- **Purpose**: Measure white noise + bias drift
- **Duration**: 60-120 seconds
- **Requirements**: Robot completely stationary
- **Measures**: Baseline noise for IMU angular velocity and linear acceleration

### Slow Spin
- **Purpose**: Measure angular velocity noise during rotation
- **Duration**: 60-90 seconds  
- **Requirements**: Slow, steady rotation (< 0.5 rad/s)
- **Measures**: IMU angular velocity noise under dynamic conditions

### Slow Straight
- **Purpose**: Measure linear acceleration noise during translation
- **Duration**: 60-90 seconds
- **Requirements**: Slow, straight motion (< 0.3 m/s)
- **Measures**: IMU linear acceleration and odometry noise during motion

## Output Format

The tool generates comprehensive, human-readable JSON files with detailed analysis information.

### IMU Results (`imu/<phase>.json`)
```json
{
  "metadata": {
    "topic": "/hardware_layer/imu/data_raw",
    "phase": "stationary",
    "analysis_timestamp": "2026-02-18T11:43:00.123456",
    "start_time": 1708243200.0,
    "end_time": 1708243260.0,
    "sample_count": 3000,
    "rate_est_hz": 100.0
  },
  "segmentation": {
    "segment_mode": "auto",
    "original_duration": 120.0,
    "original_samples": 12000,
    "trim_applied": {
      "start_seconds": 30,
      "end_seconds": 30
    },
    "threshold_applied": {},
    "final_duration": 60.0,
    "final_samples": 3000,
    "time_windows": [
      {
        "start_time": 30.0,
        "end_time": 90.0,
        "duration": 60.0
      }
    ]
  },
  "statistics": {
    "angular_velocity": {
      "x": {
        "variance": 0.001,
        "std_dev": 0.0316,
        "mean": 0.0002,
        "min": -0.095,
        "max": 0.098,
        "sample_count": 3000
      }
    }
  },
  "covariances": {
    "angular_velocity": {
      "var_x": 0.001,
      "var_y": 0.001,
      "var_z": 0.0015
    },
    "linear_acceleration": {
      "var_x": 0.01,
      "var_y": 0.01,
      "var_z": 0.02
    }
  }
}
```

### Odometry Results (`odom/<phase>.json`)
```json
{
  "metadata": {
    "topic": "/hardware_layer/diff_cont/odom",
    "phase": "stationary",
    "analysis_timestamp": "2026-02-18T11:43:00.123456",
    "start_time": 1708243200.0,
    "end_time": 1708243260.0,
    "sample_count": 1200,
    "rate_est_hz": 20.0
  },
  "segmentation": {
    "segment_mode": "auto",
    "original_duration": 120.0,
    "original_samples": 2400,
    "final_duration": 60.0,
    "final_samples": 1200
  },
  "statistics": {
    "twist": {
      "linear": {
        "x": {
          "variance": 0.001,
          "std_dev": 0.0316,
          "mean": 0.0001,
          "min": -0.05,
          "max": 0.048,
          "sample_count": 1200
        }
      },
      "angular": {
        "z": {
          "variance": 0.002,
          "std_dev": 0.0447,
          "mean": -0.0003,
          "min": -0.08,
          "max": 0.075,
          "sample_count": 1200
        }
      }
    }
  },
  "covariances": {
    "twist": {
      "linear": {
        "x_variance": 0.001
      },
      "angular": {
        "z_variance": 0.002
      }
    }
  }
}
```

### Summary Report (`report.md`)
Automatically generated report with:
- **Segmentation Analysis**: Which data segments were used and why
- **Recommended Covariance Values**: Ready-to-use values for ROS messages
- **Phase Comparison**: How noise characteristics change between phases
- **Quality Assessment**: Data quality indicators and warnings
- **Traceability**: Complete record of analysis parameters for team reviews
- **Ready-to-use YAML Configuration**: Copy-paste covariance values

## Segmentation Strategies

The tool supports different data segmentation approaches to handle various robot configurations and debugging scenarios:

### Segmentation Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| `auto` | Apply both trimming and thresholds (default) | Standard analysis for most robots |
| `trim_only` | Only time-based trimming | When velocity thresholds are unreliable |
| `threshold_only` | Only velocity thresholds | When startup/ending effects are minimal |
| `raw` | Use raw data without filtering | Debugging or when all data is needed |

### Custom Thresholds

- `--wz_min`: Minimum angular velocity for slow_spin phase (rad/s)
- `--vx_min`: Minimum linear velocity for slow_straight phase (m/s)

Different chassis may require different thresholds:
```bash
# For high-precision robots
--wz_min 0.02 --vx_min 0.01

# For larger/slower robots  
--wz_min 0.1 --vx_min 0.05

# For debugging with very slow motion
--wz_min 0.005 --vx_min 0.002
```

## Configuration

### Recording Configuration (`record/topics.yaml`)
- Define which topics to record for each phase
- Separate required, optional, and debug topics
- Phase-specific topic additions/removals

### Calculation Configuration (`calculate/config.yaml`)
- Time window settings
- Sample filtering parameters
- Output format options
- Quality thresholds

## Dependencies

- Python 3.8+
- numpy
- scipy
- rosbag2_py (for bag reading)
- pyyaml
- argparse

## Installation

```bash
pip install numpy scipy pyyaml
# ROS2 dependencies should already be available in your workspace
```
