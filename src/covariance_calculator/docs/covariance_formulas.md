# Covariance Calculation Formulas

This document describes the mathematical formulas and methods used for calculating measurement noise covariance matrices for IMU and Odometry sensors.

## Overview

The covariance calculator analyzes sensor data from three motion states:
- **Stationary**: Robot at rest (for measuring sensor noise)
- **Slow Spin**: In-place rotation (for analyzing rotational measurements)
- **Straight Line**: Linear motion (for analyzing translational measurements)

## IMU Covariance Calculation

### Target Variables
- **Yaw (ψ)**: Orientation around z-axis extracted from quaternion
- **Yaw Rate (ω_z)**: Angular velocity around z-axis

### 1. Yaw Extraction from Quaternion

Given quaternion `q = [x, y, z, w]`, yaw is calculated as:

```
yaw = atan2(2(w*z + x*y), 1 - 2(y² + z²))
```

### 2. Sample Covariance Calculation

For a dataset of N measurements {x₁, x₂, ..., xₙ}:

**Sample Mean:**
```
x̄ = (1/N) * Σᵢ₌₁ᴺ xᵢ
```

**Sample Variance:**
```
σ² = (1/(N-1)) * Σᵢ₌₁ᴺ (xᵢ - x̄)²
```

**Sample Covariance (for two variables x, y):**
```
Cov(x,y) = (1/(N-1)) * Σᵢ₌₁ᴺ (xᵢ - x̄)(yᵢ - ȳ)
```

### 3. IMU Covariance Matrix

The 2x2 covariance matrix for yaw and yaw_rate:

```
C_imu = [σ²_yaw        Cov(yaw,ω_z)    ]
        [Cov(yaw,ω_z)  σ²_yaw_rate     ]
```

### 4. Motion State Analysis

**Stationary State:**
- Yaw should remain constant → measure yaw drift/noise
- Yaw rate should be zero → measure gyroscope bias and noise

**Spin State:**
- Yaw should change linearly → measure dynamic yaw accuracy
- Yaw rate should be constant → measure gyroscope scale factor accuracy

**Straight State:**
- Yaw should remain constant → measure yaw stability during translation
- Yaw rate should be zero → measure gyroscope cross-axis sensitivity

## Odometry Covariance Calculation

### Target Variables
- **Velocity X (v_x)**: Linear velocity in x-direction (forward/backward)
- **Velocity Y (v_y)**: Linear velocity in y-direction (left/right)

### 1. Velocity Extraction

From `nav_msgs/msg/Odometry`:
```
v_x = msg.twist.twist.linear.x
v_y = msg.twist.twist.linear.y
```

### 2. Odometry Covariance Matrix

The 2x2 covariance matrix for velocity components:

```
C_odom = [σ²_vx        Cov(vx,vy)]
         [Cov(vx,vy)   σ²_vy    ]
```

### 3. Motion State Analysis

**Stationary State:**
- Both velocities should be zero → measure encoder noise and drift

**Spin State:**
- v_x should be zero → measure forward/backward velocity noise during rotation
- v_y should be zero → measure lateral velocity noise during rotation

**Straight State:**
- v_x should be constant → measure forward velocity accuracy
- v_y should be zero → measure lateral drift during straight motion

## Stationary State Detection

Robot is considered stationary when:

```
|wheel_velocity_left| < threshold AND |wheel_velocity_right| < threshold
```

Where wheel velocities are extracted from `sensor_msgs/msg/JointState`.

**Parameters:**
- `wheel_velocity_threshold`: 0.01 rad/s (configurable)
- `min_stationary_duration`: 2.0 seconds (configurable)

## Outlier Removal

Statistical outliers are removed using the standard deviation method:

```
outlier = |x - x̄| > k * σ
```

Where `k` is the threshold (default: 3.0 standard deviations).

## Robot Localization Integration

The calculated covariance matrices are formatted for direct use in `robot_localization` package:

### IMU Configuration
```yaml
imu0_config: [false, false, false,
              false, false, true,   # yaw
              false, false, false,
              false, false, true,   # yaw_rate
              false, false, false]

# Covariance matrix (6x6, only yaw and yaw_rate populated)
imu0_covariance: [0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, 0,
                  0, 0, 0, 0, 0, σ²_yaw]
```

### Odometry Configuration
```yaml
odom0_config: [false, false, false,
               false, false, false,
               true,  true,  false,  # vx, vy
               false, false, false,
               false, false, false]

# Covariance matrix (6x6, only vx and vy populated)
odom0_covariance: [0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0,
                   0, 0, 0, σ²_vx, σ²_vy, 0]
```

## Future Extensions

### Allan Variance Analysis
For more sophisticated noise characterization:

```
Allan Variance: σ²_A(τ) = (1/2) * E[(x(t+τ) - x(t))²]
```

This method can distinguish between:
- White noise
- Bias instability
- Random walk
- Rate random walk

### Spectral Analysis
Power spectral density analysis for frequency-domain noise characterization.

## References

1. IEEE Standard for Inertial Sensor Terminology (IEEE Std 528-2019)
2. "Fundamentals of Kalman Filtering: A Practical Approach" by Paul Zarchan
3. ROS robot_localization package documentation
4. Allan Variance analysis for IMU characterization
