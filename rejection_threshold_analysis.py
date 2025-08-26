#!/usr/bin/env python3
"""
Rejection threshold analysis tool
Analyze sensor data to determine optimal rejection thresholds
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import rosbag2_py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry

class ThresholdAnalyzer:
    def __init__(self):
        self.gnss_data = []
        self.imu_data = []
        self.odom_data = []
        
    def analyze_gnss_noise(self, gnss_positions):
        """
        Analyze GNSS position noise characteristics
        
        Args:
            gnss_positions: List of (lat, lon, alt) tuples
        """
        positions = np.array(gnss_positions)
        
        # Calculate position differences (innovation proxy)
        if len(positions) < 2:
            return None
            
        # Simple differencing to estimate noise
        diff_lat = np.diff(positions[:, 0]) * 111320  # Convert to meters
        diff_lon = np.diff(positions[:, 1]) * 111320 * np.cos(np.radians(positions[:-1, 1]))
        diff_alt = np.diff(positions[:, 2])
        
        # Calculate statistics
        stats_result = {
            'lat_std': np.std(diff_lat),
            'lon_std': np.std(diff_lon),
            'alt_std': np.std(diff_alt),
            'horizontal_rms': np.sqrt(np.mean(diff_lat**2 + diff_lon**2)),
            'lat_95_percentile': np.percentile(np.abs(diff_lat), 95),
            'lon_95_percentile': np.percentile(np.abs(diff_lon), 95),
        }
        
        return stats_result
    
    def analyze_imu_noise(self, angular_velocities, linear_accelerations):
        """
        Analyze IMU noise characteristics
        
        Args:
            angular_velocities: List of (wx, wy, wz) tuples
            linear_accelerations: List of (ax, ay, az) tuples
        """
        ang_vel = np.array(angular_velocities)
        lin_acc = np.array(linear_accelerations)
        
        # Remove gravity from accelerations (assuming static or slow motion)
        gravity_removed = lin_acc.copy()
        gravity_removed[:, 2] -= 9.81  # Remove gravity from Z-axis
        
        stats_result = {
            'angular_velocity': {
                'x_std': np.std(ang_vel[:, 0]),
                'y_std': np.std(ang_vel[:, 1]), 
                'z_std': np.std(ang_vel[:, 2]),
                'magnitude_95_percentile': np.percentile(np.linalg.norm(ang_vel, axis=1), 95)
            },
            'linear_acceleration': {
                'x_std': np.std(gravity_removed[:, 0]),
                'y_std': np.std(gravity_removed[:, 1]),
                'z_std': np.std(gravity_removed[:, 2]),
                'magnitude_95_percentile': np.percentile(np.linalg.norm(gravity_removed, axis=1), 95)
            }
        }
        
        return stats_result
    
    def calculate_rejection_thresholds(self, sensor_stats, confidence_level=0.95):
        """
        Calculate rejection thresholds based on sensor statistics
        
        Args:
            sensor_stats: Dictionary of sensor noise statistics
            confidence_level: Confidence level for threshold (0.90, 0.95, 0.99)
        """
        # Chi-square critical values for different confidence levels and DOF
        chi_square_table = {
            0.90: {1: 2.71, 2: 4.61, 3: 6.25},
            0.95: {1: 3.84, 2: 5.99, 3: 7.81},
            0.99: {1: 6.63, 2: 9.21, 3: 11.34},
            0.999: {1: 10.83, 2: 13.82, 3: 16.27}
        }
        
        chi_critical = chi_square_table[confidence_level]
        
        thresholds = {}
        
        # GNSS thresholds (2DOF for horizontal position)
        if 'horizontal_rms' in sensor_stats:
            # Use RMS noise as basis for threshold
            base_noise = sensor_stats['horizontal_rms']
            thresholds['gnss_pose_rejection_threshold'] = base_noise * np.sqrt(chi_critical[2])
        
        # IMU thresholds (3DOF for angular velocity)
        if 'angular_velocity' in sensor_stats:
            ang_vel_noise = sensor_stats['angular_velocity']['magnitude_95_percentile']
            thresholds['imu_twist_rejection_threshold'] = ang_vel_noise * np.sqrt(chi_critical[3])
            
        if 'linear_acceleration' in sensor_stats:
            lin_acc_noise = sensor_stats['linear_acceleration']['magnitude_95_percentile']
            thresholds['imu_linear_acceleration_rejection_threshold'] = lin_acc_noise * np.sqrt(chi_critical[3])
        
        return thresholds
    
    def generate_config_recommendations(self, thresholds):
        """
        Generate EKF configuration recommendations
        """
        config_text = f"""
# Recommended EKF rejection thresholds based on data analysis
# Generated automatically from sensor data statistics

ekf_filter_node:
  ros__parameters:
    # GNSS-based thresholds
    odom0_pose_rejection_threshold: {thresholds.get('gnss_pose_rejection_threshold', 5.0):.1f}
    odom0_twist_rejection_threshold: {thresholds.get('gnss_pose_rejection_threshold', 5.0) * 0.5:.1f}
    
    # IMU-based thresholds  
    imu0_pose_rejection_threshold: {thresholds.get('imu_twist_rejection_threshold', 0.8):.1f}
    imu0_twist_rejection_threshold: {thresholds.get('imu_twist_rejection_threshold', 0.8):.1f}
    imu0_linear_acceleration_rejection_threshold: {thresholds.get('imu_linear_acceleration_rejection_threshold', 0.8):.1f}
"""
        return config_text

def example_usage():
    """
    Example of how to use the threshold analyzer
    """
    analyzer = ThresholdAnalyzer()
    
    # Example GNSS data (latitude, longitude, altitude)
    gnss_data = [
        (56.164379, 10.145631, 114.3),
        (56.164380, 10.145632, 114.2),
        (56.164381, 10.145633, 114.4),
        (56.164379, 10.145630, 114.1),
        # ... more data points
    ]
    
    # Example IMU data
    angular_velocities = [
        (0.001, -0.002, 0.003),
        (0.002, -0.001, 0.002),
        (-0.001, 0.001, -0.001),
        # ... more data points
    ]
    
    linear_accelerations = [
        (0.1, 0.05, 9.85),
        (0.05, 0.1, 9.79),
        (-0.05, -0.1, 9.82),
        # ... more data points  
    ]
    
    # Analyze noise characteristics
    gnss_stats = analyzer.analyze_gnss_noise(gnss_data)
    imu_stats = analyzer.analyze_imu_noise(angular_velocities, linear_accelerations)
    
    print("GNSS Noise Analysis:")
    print(f"  Horizontal RMS: {gnss_stats['horizontal_rms']:.3f} m")
    print(f"  Lat 95%: {gnss_stats['lat_95_percentile']:.3f} m")
    print(f"  Lon 95%: {gnss_stats['lon_95_percentile']:.3f} m")
    
    print("\nIMU Noise Analysis:")
    print(f"  Angular velocity 95%: {imu_stats['angular_velocity']['magnitude_95_percentile']:.4f} rad/s")
    print(f"  Linear acceleration 95%: {imu_stats['linear_acceleration']['magnitude_95_percentile']:.3f} m/s²")
    
    # Calculate thresholds
    combined_stats = {**gnss_stats, **imu_stats}
    thresholds = analyzer.calculate_rejection_thresholds(combined_stats, confidence_level=0.95)
    
    print("\nRecommended Rejection Thresholds:")
    for key, value in thresholds.items():
        print(f"  {key}: {value:.2f}")
    
    # Generate config
    config = analyzer.generate_config_recommendations(thresholds)
    print("\nGenerated EKF Configuration:")
    print(config)

if __name__ == "__main__":
    example_usage()
