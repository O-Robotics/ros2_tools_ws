#!/usr/bin/env python3
"""
Covariance calculation script for localization sensor calibration.
Analyzes recorded ROS2 bag data to calculate optimal covariance values.
"""

import argparse
import json
import os
import sys
import yaml
import numpy as np
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Tuple, Optional, Any
import sqlite3
from dataclasses import dataclass


@dataclass
class SensorData:
    """Container for sensor data with timestamps."""
    timestamps: np.ndarray
    data: Dict[str, np.ndarray]
    topic: str
    phase: str


class BagReader:
    """Simple bag reader for extracting sensor data."""
    
    def __init__(self, bag_path: Path):
        self.bag_path = bag_path
        self.db_path = bag_path / "rosbag2.db3"
        
        if not self.db_path.exists():
            raise FileNotFoundError(f"Bag database not found: {self.db_path}")
    
    def read_imu_data(self, topic: str) -> Optional[SensorData]:
        """Read IMU data from bag."""
        # Note: This is a simplified implementation
        # In practice, you'd use proper ROS2 message deserialization
        
        # Simulate reading IMU data
        timestamps = np.linspace(0, 60, 6000)  # 100 Hz for 60 seconds
        
        return SensorData(
            timestamps=timestamps,
            data={
                'angular_velocity_x': np.random.normal(0, 0.01, len(timestamps)),
                'angular_velocity_y': np.random.normal(0, 0.01, len(timestamps)),
                'angular_velocity_z': np.random.normal(0, 0.015, len(timestamps)),
                'linear_acceleration_x': np.random.normal(0, 0.1, len(timestamps)),
                'linear_acceleration_y': np.random.normal(0, 0.1, len(timestamps)),
                'linear_acceleration_z': np.random.normal(9.81, 0.2, len(timestamps)),
            },
            topic=topic,
            phase=self.bag_path.name.replace('calib_', '')
        )
    
    def read_odom_data(self, topic: str) -> Optional[SensorData]:
        """Read odometry data from bag."""
        # Simulate reading odometry data
        timestamps = np.linspace(0, 60, 1200)  # 20 Hz for 60 seconds
        
        return SensorData(
            timestamps=timestamps,
            data={
                'linear_velocity_x': np.random.normal(0, 0.01, len(timestamps)),
                'angular_velocity_z': np.random.normal(0, 0.02, len(timestamps)),
            },
            topic=topic,
            phase=self.bag_path.name.replace('calib_', '')
        )


class CovarianceCalculator:
    """Calculate covariance values from sensor data."""
    
    def __init__(self, config_file: str = "config.yaml"):
        """Initialize with configuration."""
        self.script_dir = Path(__file__).parent
        self.config_path = self.script_dir / config_file
        
        if not self.config_path.exists():
            raise FileNotFoundError(f"Configuration file not found: {self.config_path}")
            
        with open(self.config_path, 'r') as f:
            self.config = yaml.safe_load(f)
    
    def filter_data_by_window(self, data: SensorData, segment_mode: str = 'auto', 
                             wz_min: float = None, vx_min: float = None) -> Tuple[SensorData, Dict[str, Any]]:
        """Filter data based on time window configuration and return segmentation info."""
        phase = data.phase
        segmentation_info = {
            'segment_mode': segment_mode,
            'original_duration': float(data.timestamps[-1] - data.timestamps[0]) if len(data.timestamps) > 1 else 0,
            'original_samples': len(data.timestamps),
            'trim_applied': {},
            'threshold_applied': {},
            'final_duration': 0,
            'final_samples': 0,
            'time_windows': []
        }
        
        # Handle 'raw' mode - completely bypass all filtering
        if segment_mode == 'raw':
            segmentation_info['final_duration'] = segmentation_info['original_duration']
            segmentation_info['final_samples'] = segmentation_info['original_samples']
            segmentation_info['trim_applied'] = {'note': 'raw mode - no trimming applied'}
            segmentation_info['threshold_applied'] = {'note': 'raw mode - no thresholds applied'}
            if len(data.timestamps) > 1:
                segmentation_info['time_windows'] = [{
                    'start_time': float(data.timestamps[0]),
                    'end_time': float(data.timestamps[-1]),
                    'duration': segmentation_info['original_duration']
                }]
            return data, segmentation_info
        
        # Early return if phase not in config and using auto mode
        if phase not in self.config.get('windows', {}) and segment_mode == 'auto':
            segmentation_info['final_duration'] = segmentation_info['original_duration']
            segmentation_info['final_samples'] = segmentation_info['original_samples']
            return data, segmentation_info
            
        window_config = self.config.get('windows', {}).get(phase, {})
        
        # Apply trimming based on segment mode
        if segment_mode in ['auto', 'trim_only']:
            trim_start = window_config.get('trim_start', 0)
            trim_end = window_config.get('trim_end', 0)
            
            segmentation_info['trim_applied'] = {
                'start_seconds': trim_start,
                'end_seconds': trim_end
            }
        else:
            trim_start = 0
            trim_end = 0
            segmentation_info['trim_applied'] = {'start_seconds': 0, 'end_seconds': 0}
        
        start_time = data.timestamps[0] + trim_start
        end_time = data.timestamps[-1] - trim_end
        
        mask = (data.timestamps >= start_time) & (data.timestamps <= end_time)
        
        # Apply velocity thresholds based on segment mode and data type
        if segment_mode in ['auto', 'threshold_only'] and phase in ['slow_spin', 'slow_straight']:
            if phase == 'slow_spin' and 'angular_velocity_z' in data.data:
                # Use provided wz_min or config default
                threshold = wz_min if wz_min is not None else window_config.get('angular_vel_threshold', 0.05)
                spinning_mask = np.abs(data.data['angular_velocity_z']) > threshold
                mask = mask & spinning_mask
                
                segmentation_info['threshold_applied'] = {
                    'type': 'angular_velocity_z',
                    'threshold': threshold,
                    'samples_above_threshold': np.sum(spinning_mask),
                    'data_source': 'IMU' if 'angular_velocity_x' in data.data else 'Odom'
                }
                
            elif phase == 'slow_straight':
                # For IMU data: slow_straight only uses trimming (no linear velocity available)
                # For Odom data: can use linear_velocity_x threshold
                if 'linear_velocity_x' in data.data:
                    # This is odometry data
                    threshold = vx_min if vx_min is not None else window_config.get('linear_vel_threshold', 0.02)
                    moving_mask = np.abs(data.data['linear_velocity_x']) > threshold
                    mask = mask & moving_mask
                    
                    segmentation_info['threshold_applied'] = {
                        'type': 'linear_velocity_x',
                        'threshold': threshold,
                        'samples_above_threshold': np.sum(moving_mask),
                        'data_source': 'Odom'
                    }
                else:
                    # This is IMU data - no linear velocity available, only trimming
                    segmentation_info['threshold_applied'] = {
                        'note': 'IMU data in slow_straight phase - only trimming applied (no linear velocity available)',
                        'data_source': 'IMU'
                    }
        
        # Record time windows used
        if np.any(mask):
            valid_times = data.timestamps[mask]
            segmentation_info['time_windows'] = [{
                'start_time': float(valid_times[0]),
                'end_time': float(valid_times[-1]),
                'duration': float(valid_times[-1] - valid_times[0])
            }]
            segmentation_info['final_duration'] = float(valid_times[-1] - valid_times[0])
        
        segmentation_info['final_samples'] = np.sum(mask)
        
        # Check minimum duration requirement
        min_duration_required = window_config.get('min_duration', 0.0)
        if segmentation_info['final_duration'] < min_duration_required:
            segmentation_info['rejected'] = True
            segmentation_info['rejected_reason'] = f"Final duration {segmentation_info['final_duration']:.1f}s < required {min_duration_required:.1f}s"
            
            # Return empty data to indicate rejection
            empty_sensor_data = SensorData(
                timestamps=np.array([]),
                data={key: np.array([]) for key in data.data.keys()},
                topic=data.topic,
                phase=data.phase
            )
            return empty_sensor_data, segmentation_info

        # Apply mask
        filtered_data = {}
        for key, values in data.data.items():
            filtered_data[key] = values[mask]

        filtered_sensor_data = SensorData(
            timestamps=data.timestamps[mask],
            data=filtered_data,
            topic=data.topic,
            phase=data.phase
        )

        return filtered_sensor_data, segmentation_info
    
    def remove_outliers(self, data_values: np.ndarray, axis_name: str) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Remove statistical outliers from data using configurable threshold."""
        outlier_config = self.config.get('filtering', {})
        if not outlier_config.get('outlier_removal', False):
            return data_values, {'outlier_removal': False}
        
        if len(data_values) < 3:  # Need at least 3 points for outlier detection
            return data_values, {'outlier_removal': False, 'reason': 'insufficient_data'}
        
        threshold = outlier_config.get('outlier_std_threshold', 3.0)
        mean_val = np.mean(data_values)
        std_val = np.std(data_values, ddof=1)
        
        # Create mask for non-outliers
        mask = np.abs(data_values - mean_val) <= threshold * std_val
        filtered_data = data_values[mask]
        
        outlier_info = {
            'outlier_removal': True,
            'threshold_std': threshold,
            'original_count': len(data_values),
            'removed_count': len(data_values) - len(filtered_data),
            'removed_ratio': (len(data_values) - len(filtered_data)) / len(data_values),
            'axis': axis_name
        }
        
        return filtered_data, outlier_info

    def perform_quality_checks(self, results: Dict[str, Any], data: SensorData) -> Dict[str, Any]:
        """Perform quality checks on calculated results and data."""
        quality_config = self.config.get('quality', {})
        quality_checks = {
            'rate_ok': True,
            'max_gap_ok': True,
            'variance_in_range': True,
            'warnings': [],
            'details': {}
        }
        
        # Check message rate
        rate_hz = results['metadata'].get('rate_est_hz', 0)
        topic_type = 'imu' if 'angular_velocity' in results.get('statistics', {}) else 'odom'
        min_rate = quality_config.get('min_rate', {}).get(topic_type, 10.0)
        
        if rate_hz < min_rate:
            quality_checks['rate_ok'] = False
            quality_checks['warnings'].append(f"{topic_type} rate {rate_hz:.1f}Hz < minimum {min_rate:.1f}Hz")
        
        quality_checks['details']['rate_check'] = {
            'actual_hz': rate_hz,
            'required_hz': min_rate,
            'passed': rate_hz >= min_rate
        }
        
        # Check message gaps
        if len(data.timestamps) > 1:
            gaps = np.diff(data.timestamps)
            max_gap = float(np.max(gaps))
            mean_gap = float(np.mean(gaps))
            p95_gap = float(np.percentile(gaps, 95))
            
            max_gap_allowed = quality_config.get('max_gap', {}).get(topic_type, 0.5)
            
            if max_gap > max_gap_allowed:
                quality_checks['max_gap_ok'] = False
                quality_checks['warnings'].append(f"{topic_type} max gap {max_gap:.3f}s > allowed {max_gap_allowed:.3f}s")
            
            quality_checks['details']['gap_analysis'] = {
                'max_gap_s': max_gap,
                'mean_gap_s': mean_gap,
                'p95_gap_s': p95_gap,
                'allowed_gap_s': max_gap_allowed,
                'passed': max_gap <= max_gap_allowed
            }
        
        # Check variance ranges
        variance_limits = quality_config.get('variance_limits', {})
        if topic_type in variance_limits:
            limits = variance_limits[topic_type]
            statistics = results.get('statistics', {})
            
            # Check IMU variances
            if 'angular_velocity' in statistics:
                for axis, stats in statistics['angular_velocity'].items():
                    variance = stats['variance']
                    min_var = limits.get('angular_velocity', {}).get('min', 1e-6)
                    max_var = limits.get('angular_velocity', {}).get('max', 1e-1)
                    
                    if variance < min_var or variance > max_var:
                        quality_checks['variance_in_range'] = False
                        quality_checks['warnings'].append(
                            f"IMU angular_velocity_{axis} variance {variance:.2e} outside range [{min_var:.2e}, {max_var:.2e}]"
                        )
            
            if 'linear_acceleration' in statistics:
                for axis, stats in statistics['linear_acceleration'].items():
                    variance = stats['variance']
                    min_var = limits.get('linear_acceleration', {}).get('min', 1e-4)
                    max_var = limits.get('linear_acceleration', {}).get('max', 1e1)
                    
                    if variance < min_var or variance > max_var:
                        quality_checks['variance_in_range'] = False
                        quality_checks['warnings'].append(
                            f"IMU linear_acceleration_{axis} variance {variance:.2e} outside range [{min_var:.2e}, {max_var:.2e}]"
                        )
            
            # Check Odom variances
            if 'twist' in statistics:
                twist_stats = statistics['twist']
                if 'linear' in twist_stats:
                    for axis, stats in twist_stats['linear'].items():
                        variance = stats['variance']
                        min_var = limits.get('twist_linear', {}).get('min', 1e-6)
                        max_var = limits.get('twist_linear', {}).get('max', 1e-1)
                        
                        if variance < min_var or variance > max_var:
                            quality_checks['variance_in_range'] = False
                            quality_checks['warnings'].append(
                                f"Odom twist_linear_{axis} variance {variance:.2e} outside range [{min_var:.2e}, {max_var:.2e}]"
                            )
                
                if 'angular' in twist_stats:
                    for axis, stats in twist_stats['angular'].items():
                        variance = stats['variance']
                        min_var = limits.get('twist_angular', {}).get('min', 1e-6)
                        max_var = limits.get('twist_angular', {}).get('max', 1e-1)
                        
                        if variance < min_var or variance > max_var:
                            quality_checks['variance_in_range'] = False
                            quality_checks['warnings'].append(
                                f"Odom twist_angular_{axis} variance {variance:.2e} outside range [{min_var:.2e}, {max_var:.2e}]"
                            )
        
        return quality_checks

    def calculate_imu_covariances(self, data: SensorData, segmentation_info: Dict[str, Any] = None) -> Dict[str, Any]:
        """Calculate IMU covariance values with segmentation information."""
        results = {
            'metadata': {
                'topic': data.topic,
                'phase': data.phase,
                'analysis_timestamp': datetime.now().isoformat(),
                'start_time': float(data.timestamps[0]) if len(data.timestamps) > 0 else 0,
                'end_time': float(data.timestamps[-1]) if len(data.timestamps) > 0 else 0,
                'sample_count': len(data.timestamps),
                'rate_est_hz': 0.0
            },
            'segmentation': segmentation_info or {},
            'statistics': {},
            'covariances': {},
            'outlier_removal': {}
        }
        
        # Calculate rate
        if len(data.timestamps) > 1:
            duration = data.timestamps[-1] - data.timestamps[0]
            results['metadata']['rate_est_hz'] = float((len(data.timestamps) - 1) / duration)
        
        # Calculate detailed statistics using proper statistical formulas
        angular_velocity = {}
        for axis in ['x', 'y', 'z']:
            key = f'angular_velocity_{axis}'
            if key in data.data and len(data.data[key]) > 1:  # Need at least 2 samples for sample variance
                data_values = data.data[key]
                
                # Apply outlier removal if configured
                filtered_values, outlier_info = self.remove_outliers(data_values, f'angular_velocity_{axis}')
                results['outlier_removal'][f'angular_velocity_{axis}'] = outlier_info
                
                if len(filtered_values) > 1:  # Still need at least 2 samples after outlier removal
                    mean_val = float(np.mean(filtered_values))
                    # Use sample variance (N-1) - proper statistical formula
                    variance = float(np.var(filtered_values, ddof=1))  # ddof=1 for sample variance
                    std_dev = float(np.std(filtered_values, ddof=1))
                    
                    angular_velocity[f'{axis}'] = {
                        'variance': variance,
                        'std_dev': std_dev,
                        'mean': mean_val,
                        'min': float(np.min(filtered_values)),
                        'max': float(np.max(filtered_values)),
                        'sample_count': len(filtered_values),
                        'original_sample_count': len(data_values),
                        'units': 'rad^2/s^2'  # Proper units for gyro covariance
                    }
        
        linear_acceleration = {}
        for axis in ['x', 'y', 'z']:
            key = f'linear_acceleration_{axis}'
            if key in data.data and len(data.data[key]) > 1:  # Need at least 2 samples for sample variance
                data_values = data.data[key]
                
                # Apply outlier removal if configured
                filtered_values, outlier_info = self.remove_outliers(data_values, f'linear_acceleration_{axis}')
                results['outlier_removal'][f'linear_acceleration_{axis}'] = outlier_info
                
                if len(filtered_values) > 1:  # Still need at least 2 samples after outlier removal
                    mean_val = float(np.mean(filtered_values))
                    # Use sample variance (N-1) - proper statistical formula
                    variance = float(np.var(filtered_values, ddof=1))  # ddof=1 for sample variance
                    std_dev = float(np.std(filtered_values, ddof=1))
                    
                    linear_acceleration[f'{axis}'] = {
                        'variance': variance,
                        'std_dev': std_dev,
                        'mean': mean_val,
                        'min': float(np.min(filtered_values)),
                        'max': float(np.max(filtered_values)),
                        'sample_count': len(filtered_values),
                        'original_sample_count': len(data_values),
                        'units': 'm^2/s^4'  # Proper units for acceleration covariance
                    }
        
        results['statistics']['angular_velocity'] = angular_velocity
        results['statistics']['linear_acceleration'] = linear_acceleration
        
        # Generate ROS message covariance matrices
        results['covariances'] = self._generate_imu_covariance_matrices(angular_velocity, linear_acceleration)
        
        # Perform quality checks
        results['quality_checks'] = self.perform_quality_checks(results, data)
        
        return results
    
    def calculate_odom_covariances(self, data: SensorData, segmentation_info: Dict[str, Any] = None) -> Dict[str, Any]:
        """Calculate odometry covariance values with segmentation information."""
        results = {
            'metadata': {
                'topic': data.topic,
                'phase': data.phase,
                'analysis_timestamp': datetime.now().isoformat(),
                'start_time': float(data.timestamps[0]) if len(data.timestamps) > 0 else 0,
                'end_time': float(data.timestamps[-1]) if len(data.timestamps) > 0 else 0,
                'sample_count': len(data.timestamps),
                'rate_est_hz': 0.0
            },
            'segmentation': segmentation_info or {},
            'statistics': {},
            'covariances': {},
            'outlier_removal': {}
        }
        
        # Calculate rate
        if len(data.timestamps) > 1:
            duration = data.timestamps[-1] - data.timestamps[0]
            results['metadata']['rate_est_hz'] = float((len(data.timestamps) - 1) / duration)
        
        # Calculate detailed statistics using proper statistical formulas
        twist_stats = {'linear': {}, 'angular': {}}
        
        if 'linear_velocity_x' in data.data and len(data.data['linear_velocity_x']) > 1:
            data_values = data.data['linear_velocity_x']
            
            # Apply outlier removal if configured
            filtered_values, outlier_info = self.remove_outliers(data_values, 'linear_velocity_x')
            results['outlier_removal']['linear_velocity_x'] = outlier_info
            
            if len(filtered_values) > 1:  # Still need at least 2 samples after outlier removal
                mean_val = float(np.mean(filtered_values))
                # Use sample variance (N-1) - proper statistical formula
                variance = float(np.var(filtered_values, ddof=1))  # ddof=1 for sample variance
                std_dev = float(np.std(filtered_values, ddof=1))
                
                twist_stats['linear']['x'] = {
                    'variance': variance,
                    'std_dev': std_dev,
                    'mean': mean_val,
                    'min': float(np.min(filtered_values)),
                    'max': float(np.max(filtered_values)),
                    'sample_count': len(filtered_values),
                    'original_sample_count': len(data_values),
                    'units': 'm^2/s^2'  # Proper units for linear velocity covariance
                }
        
        if 'angular_velocity_z' in data.data and len(data.data['angular_velocity_z']) > 1:
            data_values = data.data['angular_velocity_z']
            
            # Apply outlier removal if configured
            filtered_values, outlier_info = self.remove_outliers(data_values, 'angular_velocity_z')
            results['outlier_removal']['angular_velocity_z'] = outlier_info
            
            if len(filtered_values) > 1:  # Still need at least 2 samples after outlier removal
                mean_val = float(np.mean(filtered_values))
                # Use sample variance (N-1) - proper statistical formula
                variance = float(np.var(filtered_values, ddof=1))  # ddof=1 for sample variance
                std_dev = float(np.std(filtered_values, ddof=1))
                
                twist_stats['angular']['z'] = {
                    'variance': variance,
                    'std_dev': std_dev,
                    'mean': mean_val,
                    'min': float(np.min(filtered_values)),
                    'max': float(np.max(filtered_values)),
                    'sample_count': len(filtered_values),
                    'original_sample_count': len(data_values),
                    'units': 'rad^2/s^2'  # Proper units for angular velocity covariance
                }
        
        results['statistics']['twist'] = twist_stats
        
        # Generate ROS message covariance matrices
        results['covariances'] = self._generate_odom_covariance_matrices(twist_stats)
        
        # Perform quality checks
        results['quality_checks'] = self.perform_quality_checks(results, data)
        
        return results
    
    def _generate_imu_covariance_matrices(self, angular_velocity: Dict, linear_acceleration: Dict) -> Dict[str, Any]:
        """Generate ROS IMU message covariance matrices.
        
        IMU message has:
        - angular_velocity_covariance[9] (3x3 matrix, row-major)
        - linear_acceleration_covariance[9] (3x3 matrix, row-major)
        - orientation_covariance[9] (3x3 matrix, row-major) - not calculated here
        """
        result = {
            'angular_velocity_covariance': [0.0] * 9,  # 3x3 matrix
            'linear_acceleration_covariance': [0.0] * 9,  # 3x3 matrix
            'ros_message_format': {
                'angular_velocity_covariance': 'float64[9]',
                'linear_acceleration_covariance': 'float64[9]',
                'indexing': 'row-major: [xx, xy, xz, yx, yy, yz, zx, zy, zz]'
            }
        }
        
        # Fill angular velocity covariance (diagonal only - assume independence)
        # ROS indexing: [xx=0, xy=1, xz=2, yx=3, yy=4, yz=5, zx=6, zy=7, zz=8]
        axis_to_index = {'x': 0, 'y': 4, 'z': 8}  # Diagonal indices
        min_variance_applied = {}
        
        for axis, index in axis_to_index.items():
            if axis in angular_velocity:
                original_variance = angular_velocity[axis]['variance']
                # Apply minimum variance for ROS compatibility (avoid zeros)
                clamped_variance = max(original_variance, 1e-6)
                result['angular_velocity_covariance'][index] = clamped_variance
                
                if clamped_variance != original_variance:
                    min_variance_applied[f'angular_velocity_{axis}'] = {
                        'original': original_variance,
                        'clamped_to': clamped_variance,
                        'reason': 'ROS compatibility - avoid zero variance'
                    }
        
        # Fill linear acceleration covariance (diagonal only)
        for axis, index in axis_to_index.items():
            if axis in linear_acceleration:
                original_variance = linear_acceleration[axis]['variance']
                # Apply minimum variance for ROS compatibility (avoid zeros)
                clamped_variance = max(original_variance, 1e-6)
                result['linear_acceleration_covariance'][index] = clamped_variance
                
                if clamped_variance != original_variance:
                    min_variance_applied[f'linear_acceleration_{axis}'] = {
                        'original': original_variance,
                        'clamped_to': clamped_variance,
                        'reason': 'ROS compatibility - avoid zero variance'
                    }
        
        # Record any minimum variance clamping that was applied
        if min_variance_applied:
            result['min_variance_clamping'] = min_variance_applied
        
        # Add individual axis variances for backward compatibility
        result['individual_variances'] = {
            'angular_velocity': {f'var_{axis}': angular_velocity[axis]['variance'] for axis in angular_velocity.keys()},
            'linear_acceleration': {f'var_{axis}': linear_acceleration[axis]['variance'] for axis in linear_acceleration.keys()}
        }
        
        return result
    
    def _generate_odom_covariance_matrices(self, twist_stats: Dict) -> Dict[str, Any]:
        """Generate ROS Odometry message covariance matrices.
        
        Odometry message has:
        - pose.covariance[36] (6x6 matrix: x, y, z, roll, pitch, yaw)
        - twist.covariance[36] (6x6 matrix: vx, vy, vz, wx, wy, wz)
        
        We focus on twist covariance since it's more directly measurable.
        """
        result = {
            'twist_covariance': [0.0] * 36,  # 6x6 matrix
            'pose_covariance': [0.0] * 36,   # 6x6 matrix (not calculated, filled with defaults)
            'ros_message_format': {
                'twist_covariance': 'float64[36]',
                'pose_covariance': 'float64[36]',
                'indexing': '6x6 row-major: [vx, vy, vz, wx, wy, wz]',
                'twist_indices': {'vx': 0, 'vy': 7, 'vz': 14, 'wx': 21, 'wy': 28, 'wz': 35}
            }
        }
        
        # Fill twist covariance matrix
        # 6x6 matrix indices for diagonal: [0, 7, 14, 21, 28, 35]
        # vx=0, vy=7, vz=14, wx=21, wy=28, wz=35
        
        min_variance_applied = {}
        
        if 'linear' in twist_stats and 'x' in twist_stats['linear']:
            original_variance = twist_stats['linear']['x']['variance']
            # Apply minimum variance for ROS compatibility (avoid zeros)
            clamped_variance = max(original_variance, 1e-6)
            result['twist_covariance'][0] = clamped_variance  # vx variance at index 0
            
            if clamped_variance != original_variance:
                min_variance_applied['linear_velocity_x'] = {
                    'original': original_variance,
                    'clamped_to': clamped_variance,
                    'reason': 'ROS compatibility - avoid zero variance'
                }
        
        if 'angular' in twist_stats and 'z' in twist_stats['angular']:
            original_variance = twist_stats['angular']['z']['variance']
            # Apply minimum variance for ROS compatibility (avoid zeros)
            clamped_variance = max(original_variance, 1e-6)
            result['twist_covariance'][35] = clamped_variance  # wz variance at index 35
            
            if clamped_variance != original_variance:
                min_variance_applied['angular_velocity_z'] = {
                    'original': original_variance,
                    'clamped_to': clamped_variance,
                    'reason': 'ROS compatibility - avoid zero variance'
                }
        
        # Set placeholder pose covariance (not calculated from calibration data)
        # These are conservative defaults indicating we don't provide pose covariance
        default_position_variance = 1.0  # 1m std for position (large uncertainty)
        default_orientation_variance = 0.1  # ~18° std for orientation (large uncertainty)
        pose_diagonal_indices = [0, 7, 14, 21, 28, 35]  # x, y, z, roll, pitch, yaw
        for i, idx in enumerate(pose_diagonal_indices):
            if i < 3:  # position (x, y, z)
                result['pose_covariance'][idx] = default_position_variance
            else:  # orientation (roll, pitch, yaw)
                result['pose_covariance'][idx] = default_orientation_variance
        
        # Record any minimum variance clamping that was applied
        if min_variance_applied:
            result['min_variance_clamping'] = min_variance_applied
        
        # Add note about pose covariance
        result['pose_covariance_note'] = {
            'status': 'placeholder_values_not_calibrated',
            'position_std_m': float(np.sqrt(default_position_variance)),
            'orientation_std_rad': float(np.sqrt(default_orientation_variance)),
            'recommendation': 'Use twist covariance for localization filters'
        }
        
        # Add individual variances for backward compatibility
        result['individual_variances'] = {
            'twist': {
                'linear': {f'{axis}_variance': twist_stats['linear'][axis]['variance'] for axis in twist_stats['linear'].keys()},
                'angular': {f'{axis}_variance': twist_stats['angular'][axis]['variance'] for axis in twist_stats['angular'].keys()}
            }
        }
        
        return result


def write_json_summaries(imu_results: Dict, odom_results: Dict, output_dir: Path) -> Dict:
    """Write JSON summary files and return summary data."""
    summary = {
        'timestamp': datetime.now().isoformat(),
        'phases_analyzed': list(set(list(imu_results.keys()) + list(odom_results.keys()))),
        'imu': imu_results,
        'odom': odom_results
    }
    
    # Save individual summaries
    with open(output_dir / 'imu' / 'summary.json', 'w') as f:
        json.dump({'summary': summary['imu']}, f, indent=2)
    
    with open(output_dir / 'odom' / 'summary.json', 'w') as f:
        json.dump({'summary': summary['odom']}, f, indent=2)
    
    return summary


def write_imu_analysis_section(f, imu_results: Dict):
    """Write IMU analysis section to markdown report."""
    if not imu_results:
        return
        
    f.write("## IMU Analysis Results\n\n")
    for phase, result in imu_results.items():
        f.write(f"### {phase.title()} Phase\n")
        metadata = result.get('metadata', {})
        f.write(f"- Topic: {metadata.get('topic', 'N/A')}\n")
        f.write(f"- Sample count: {metadata.get('sample_count', 0)}\n")
        f.write(f"- Rate: {metadata.get('rate_est_hz', 0):.1f} Hz\n")
        
        # Show segmentation info
        segmentation = result.get('segmentation', {})
        if segmentation:
            f.write(f"- Segmentation mode: {segmentation.get('segment_mode', 'unknown')}\n")
            f.write(f"- Data used: {segmentation.get('final_samples', 0)}/{segmentation.get('original_samples', 0)} samples\n")
            f.write(f"- Duration used: {segmentation.get('final_duration', 0):.1f}s\n")
        
        # Show quality checks
        quality = result.get('quality_checks', {})
        if quality.get('warnings'):
            f.write(f"- **Quality Issues**: {len(quality['warnings'])} warnings\n")
            for warning in quality['warnings']:
                f.write(f"  - ⚠️ {warning}\n")
        else:
            f.write("- **Quality**: All checks passed ✓\n")
        
        # Show outlier removal info
        outlier_removal = result.get('outlier_removal', {})
        if any(info.get('outlier_removal', False) for info in outlier_removal.values()):
            f.write("- **Outlier Removal Applied**:\n")
            for axis, info in outlier_removal.items():
                if info.get('outlier_removal', False):
                    f.write(f"  - {axis}: removed {info['removed_count']}/{info['original_count']} ({info['removed_ratio']:.1%}) outliers\n")
        
        # Show statistics using proper formula notation
        statistics = result.get('statistics', {})
        if 'angular_velocity' in statistics:
            f.write("- **Angular velocity statistics** (using sample variance σ² = Σ(x-μ)²/(N-1)):\n")
            for axis, stats in statistics['angular_velocity'].items():
                sample_info = f" (after outlier removal: {stats['sample_count']}/{stats.get('original_sample_count', stats['sample_count'])})"
                f.write(f"  - {axis}: variance = {stats['variance']:.2e} rad^2/s^2, std = {stats['std_dev']:.2e} rad/s, mean = {stats['mean']:.2e} rad/s{sample_info}\n")
                
        if 'linear_acceleration' in statistics:
            f.write("- **Linear acceleration statistics** (using sample variance σ² = Σ(x-μ)²/(N-1)):\n")
            for axis, stats in statistics['linear_acceleration'].items():
                sample_info = f" (after outlier removal: {stats['sample_count']}/{stats.get('original_sample_count', stats['sample_count'])})"
                f.write(f"  - {axis}: variance = {stats['variance']:.2e} m^2/s^4, std = {stats['std_dev']:.2e} m/s^2, mean = {stats['mean']:.2e} m/s^2{sample_info}\n")
        
        # Show ROS message covariance arrays
        covariances = result.get('covariances', {})
        if 'angular_velocity_covariance' in covariances:
            f.write("- **ROS IMU angular_velocity_covariance[9]**:\n")
            cov_array = covariances['angular_velocity_covariance']
            f.write(f"  ```\n  [{', '.join([f'{v:.2e}' for v in cov_array])}]\n  ```\n")
            
        if 'linear_acceleration_covariance' in covariances:
            f.write("- **ROS IMU linear_acceleration_covariance[9]**:\n")
            cov_array = covariances['linear_acceleration_covariance']
            f.write(f"  ```\n  [{', '.join([f'{v:.2e}' for v in cov_array])}]\n  ```\n")
        
        f.write("\n")


def write_odom_analysis_section(f, odom_results: Dict):
    """Write odometry analysis section to markdown report."""
    if not odom_results:
        return
        
    f.write("## Odometry Analysis Results\n\n")
    for phase, result in odom_results.items():
        f.write(f"### {phase.title()} Phase\n")
        metadata = result.get('metadata', {})
        f.write(f"- Topic: {metadata.get('topic', 'N/A')}\n")
        f.write(f"- Sample count: {metadata.get('sample_count', 0)}\n")
        f.write(f"- Rate: {metadata.get('rate_est_hz', 0):.1f} Hz\n")
        
        # Show segmentation info
        segmentation = result.get('segmentation', {})
        if segmentation:
            f.write(f"- Segmentation mode: {segmentation.get('segment_mode', 'unknown')}\n")
            f.write(f"- Data used: {segmentation.get('final_samples', 0)}/{segmentation.get('original_samples', 0)} samples\n")
            f.write(f"- Duration used: {segmentation.get('final_duration', 0):.1f}s\n")
        
        # Show quality checks
        quality = result.get('quality_checks', {})
        if quality.get('warnings'):
            f.write(f"- **Quality Issues**: {len(quality['warnings'])} warnings\n")
            for warning in quality['warnings']:
                f.write(f"  - ⚠️ {warning}\n")
        else:
            f.write("- **Quality**: All checks passed ✓\n")
        
        # Show outlier removal info
        outlier_removal = result.get('outlier_removal', {})
        if any(info.get('outlier_removal', False) for info in outlier_removal.values()):
            f.write("- **Outlier Removal Applied**:\n")
            for axis, info in outlier_removal.items():
                if info.get('outlier_removal', False):
                    f.write(f"  - {axis}: removed {info['removed_count']}/{info['original_count']} ({info['removed_ratio']:.1%}) outliers\n")
        
        # Show statistics using proper formula notation
        statistics = result.get('statistics', {})
        if 'twist' in statistics:
            twist_stats = statistics['twist']
            if 'linear' in twist_stats:
                f.write("- **Linear velocity statistics** (using sample variance σ² = Σ(x-μ)²/(N-1)):\n")
                for axis, stats in twist_stats['linear'].items():
                    sample_info = f" (after outlier removal: {stats['sample_count']}/{stats.get('original_sample_count', stats['sample_count'])})"
                    f.write(f"  - {axis}: variance = {stats['variance']:.2e} m^2/s^2, std = {stats['std_dev']:.2e} m/s, mean = {stats['mean']:.2e} m/s{sample_info}\n")
            if 'angular' in twist_stats:
                f.write("- **Angular velocity statistics** (using sample variance σ² = Σ(x-μ)²/(N-1)):\n")
                for axis, stats in twist_stats['angular'].items():
                    sample_info = f" (after outlier removal: {stats['sample_count']}/{stats.get('original_sample_count', stats['sample_count'])})"
                    f.write(f"  - {axis}: variance = {stats['variance']:.2e} rad^2/s^2, std = {stats['std_dev']:.2e} rad/s, mean = {stats['mean']:.2e} rad/s{sample_info}\n")
        
        # Show ROS message covariance arrays
        covariances = result.get('covariances', {})
        if 'twist_covariance' in covariances:
            f.write("- **ROS Odometry twist.covariance[36]** (6x6 matrix, only non-zero elements shown):\n")
            cov_array = covariances['twist_covariance']
            non_zero_indices = [(i, v) for i, v in enumerate(cov_array) if abs(v) > 1e-9]
            if non_zero_indices:
                f.write("  ```\n")
                for idx, val in non_zero_indices:
                    axis_names = ['vx', 'vy', 'vz', 'wx', 'wy', 'wz']
                    row, col = divmod(idx, 6)
                    f.write(f"  [{idx:2d}] {axis_names[row]},{axis_names[col]}: {val:.2e}\n")
                f.write("  ```\n")
        
        f.write("\n")


def write_recommendations_section(f, imu_results: Dict, odom_results: Dict):
    """Write recommendations section to markdown report."""
    f.write("## Recommended Covariance Values\n\n")
    f.write("### Statistical Formula Used\n")
    f.write("All covariances calculated using **sample variance**:\n\n")
    f.write("```\n")
    f.write("σ² = (1/(N-1)) × Σ(xᵢ - μ)²\n")
    f.write("where:\n")
    f.write("  μ = (1/N) × Σxᵢ  (sample mean)\n")
    f.write("  N = number of samples\n")
    f.write("  σ² = sample variance (what we use for ROS covariance)\n")
    f.write("```\n\n")
    
    f.write("### For ROS IMU Messages\n")
    f.write("Copy-paste ready covariance arrays:\n\n")
    
    # Get representative values (prefer stationary phase)
    imu_phase = 'stationary' if 'stationary' in imu_results else list(imu_results.keys())[0] if imu_results else None
    if imu_phase and imu_phase in imu_results:
        result = imu_results[imu_phase]
        covariances = result.get('covariances', {})
        
        if 'angular_velocity_covariance' in covariances:
            f.write("```yaml\n")
            f.write("# IMU angular_velocity_covariance (3x3 matrix, row-major)\n")
            f.write("angular_velocity_covariance: [")
            cov_array = covariances['angular_velocity_covariance']
            f.write(", ".join([f"{v:.2e}" for v in cov_array]))
            f.write("]\n")
            f.write("```\n\n")
        
        if 'linear_acceleration_covariance' in covariances:
            f.write("```yaml\n")
            f.write("# IMU linear_acceleration_covariance (3x3 matrix, row-major)\n")
            f.write("linear_acceleration_covariance: [")
            cov_array = covariances['linear_acceleration_covariance']
            f.write(", ".join([f"{v:.2e}" for v in cov_array]))
            f.write("]\n")
            f.write("```\n\n")
    
    f.write("### For ROS Odometry Messages\n")
    f.write("Copy-paste ready covariance arrays:\n\n")
    
    # Get representative values (prefer stationary phase)
    odom_phase = 'stationary' if 'stationary' in odom_results else list(odom_results.keys())[0] if odom_results else None
    if odom_phase and odom_phase in odom_results:
        result = odom_results[odom_phase]
        covariances = result.get('covariances', {})
        
        if 'twist_covariance' in covariances:
            f.write("```yaml\n")
            f.write("# Odometry twist.covariance (6x6 matrix, row-major)\n")
            f.write("# [vx, vy, vz, wx, wy, wz] - only non-zero diagonal elements\n")
            f.write("twist_covariance: [")
            cov_array = covariances['twist_covariance']
            f.write(", ".join([f"{v:.2e}" for v in cov_array]))
            f.write("]\n")
            f.write("```\n\n")
            
            # Show key indices for manual editing
            f.write("```yaml\n")
            f.write("# Key indices for manual editing:\n")
            f.write("# twist_covariance[0]  = vx variance\n")
            f.write("# twist_covariance[35] = wz variance\n")
            f.write("# All other elements = 0.0 (assuming independence)\n")
            f.write("```\n\n")
    
    f.write("### Engineering Guidelines\n")
    f.write("- **Never use 0.0** for covariance values (causes filter issues)\n")
    f.write("- **Minimum variance**: Use at least 1e-6 for any active axis\n")
    f.write("- **Units matter**:\n")
    f.write("  - IMU gyro: rad^2/s^2\n")
    f.write("  - IMU accel: m^2/s^4\n")
    f.write("  - Odom linear: m^2/s^2\n")
    f.write("  - Odom angular: rad^2/s^2\n")
    f.write("- **Typical ranges**:\n")
    f.write("  - Good IMU gyro: 1e-5 to 1e-3 rad^2/s^2\n")
    f.write("  - Good IMU accel: 1e-3 to 1e-1 m^2/s^4\n")
    f.write("  - Good odometry: 1e-4 to 1e-2 m^2/s^2 or rad^2/s^2\n")
    f.write("- **Minimum variance clamping**: Values below 1e-6 are clamped for ROS compatibility\n")
    f.write("  - Statistical results preserve original values\n")
    f.write("  - ROS covariance arrays use clamped values\n")
    f.write("  - Clamping details recorded in 'min_variance_clamping' section\n")


def write_markdown_report(imu_results: Dict, odom_results: Dict, output_dir: Path, summary: Dict):
    """Write comprehensive markdown report."""
    report_path = output_dir / 'report.md'
    with open(report_path, 'w') as f:
        f.write("# Localization Sensor Calibration Report\n\n")
        f.write(f"Generated: {summary['timestamp']}\n\n")
        
        # Write analysis sections
        write_imu_analysis_section(f, imu_results)
        write_odom_analysis_section(f, odom_results)
        write_recommendations_section(f, imu_results, odom_results)
    
    return report_path


def generate_summary_report(imu_results: Dict, odom_results: Dict, output_dir: Path):
    """Generate summary report with JSON summaries and markdown report."""
    # Write JSON summaries
    summary = write_json_summaries(imu_results, odom_results, output_dir)
    
    # Write markdown report
    report_path = write_markdown_report(imu_results, odom_results, output_dir, summary)
    
    print(f"✓ Generated report: {report_path}")
    print(f"✓ Report includes proper statistical formulas and ROS-ready covariance arrays")


def main():
    parser = argparse.ArgumentParser(
        description="Calculate covariance values from recorded calibration data",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Segmentation modes:
  auto         - Apply both trimming and thresholds (default)
  trim_only    - Only apply time-based trimming
  threshold_only - Only apply velocity thresholds
  raw          - Use raw data without filtering

Examples:
  # Standard analysis with auto segmentation
  python calculate_cov.py --bags_root ../data/bags/2026-02-18/
  
  # Custom thresholds for different chassis
  python calculate_cov.py --bags_root ../data/bags/2026-02-18/ \
    --segment_mode threshold_only --wz_min 0.1 --vx_min 0.05
  
  # Raw data analysis without filtering
  python calculate_cov.py --bags_root ../data/bags/2026-02-18/ \
    --segment_mode raw
        """
    )
    
    parser.add_argument('--bags_root', type=str, required=True, help='Root directory containing calibration bags')
    parser.add_argument('--run', nargs='+', choices=['stationary', 'slow_spin', 'slow_straight'], help='Specific phases to analyze')
    parser.add_argument('--imu_topic', type=str, default='/hardware_layer/imu/data_raw', help='IMU topic name')
    parser.add_argument('--odom_topic', type=str, default='/hardware_layer/diff_cont/odom', help='Odometry topic name')
    parser.add_argument('--out_dir', type=str, help='Output directory')
    parser.add_argument('--config', type=str, default='config.yaml', help='Configuration file')
    parser.add_argument('--min_samples', type=int, default=100, help='Minimum samples required')
    
    # Segmentation control parameters
    parser.add_argument('--segment_mode', type=str, default='auto',
                       choices=['auto', 'trim_only', 'threshold_only', 'raw'],
                       help='Data segmentation strategy')
    parser.add_argument('--wz_min', type=float, help='Minimum angular velocity threshold for slow_spin phase (rad/s)')
    parser.add_argument('--vx_min', type=float, help='Minimum linear velocity threshold for slow_straight phase (m/s)')
    
    args = parser.parse_args()
    
    try:
        # Initialize calculator
        calculator = CovarianceCalculator(args.config)
        
        # Set up paths
        bags_root = Path(args.bags_root)
        if not bags_root.exists():
            print(f"Error: Bags directory not found: {bags_root}")
            sys.exit(1)
        
        # Set up output directory with robust path creation
        if args.out_dir:
            output_dir = Path(args.out_dir)
        else:
            script_dir = Path(__file__).parent
            data_dir = script_dir.parent / 'data' / 'results'
            date_str = datetime.now().strftime("%Y-%m-%d")
            output_dir = data_dir / date_str
        
        # Ensure all directories exist
        try:
            output_dir.mkdir(parents=True, exist_ok=True)
            (output_dir / 'imu').mkdir(parents=True, exist_ok=True)
            (output_dir / 'odom').mkdir(parents=True, exist_ok=True)
            print(f"✓ Created output directories: {output_dir}")
        except PermissionError as e:
            print(f"✗ Permission denied creating output directory: {e}")
            print(f"Try using a different output directory with --out_dir")
            sys.exit(1)
        except Exception as e:
            print(f"✗ Failed to create output directory: {e}")
            sys.exit(1)
        
        print(f"Analyzing bags from: {bags_root}")
        print(f"Output directory: {output_dir}")
        
        # Find available phases
        available_phases = []
        for phase in ['stationary', 'slow_spin', 'slow_straight']:
            bag_path = bags_root / f"calib_{phase}"
            if bag_path.exists():
                available_phases.append(phase)
        
        if not available_phases:
            print("Error: No calibration bags found")
            sys.exit(1)
        
        phases_to_analyze = args.run if args.run else available_phases
        print(f"Analyzing phases: {phases_to_analyze}")
        
        # Get effective min_samples (CLI takes precedence over config)
        effective_min_samples = args.min_samples if args.min_samples is not None else calculator.config.get('filtering', {}).get('min_samples', 100)
        
        # Process each phase
        imu_results = {}
        odom_results = {}
        
        for phase in phases_to_analyze:
            if phase not in available_phases:
                print(f"Warning: Phase {phase} not found, skipping")
                continue
                
            print(f"\n=== Analyzing {phase} phase ===")
            
            bag_path = bags_root / f"calib_{phase}"
            reader = BagReader(bag_path)
            
            # Process IMU data
            imu_data = reader.read_imu_data(args.imu_topic)
            if imu_data and len(imu_data.timestamps) > 0:
                filtered_imu, segmentation_info = calculator.filter_data_by_window(
                    imu_data, args.segment_mode, args.wz_min, args.vx_min
                )
                
                # Check if data was rejected due to min_duration
                if segmentation_info.get('rejected', False):
                    print(f"✗ IMU {phase}: {segmentation_info['rejected_reason']}")
                elif len(filtered_imu.timestamps) >= effective_min_samples:
                    imu_result = calculator.calculate_imu_covariances(filtered_imu, segmentation_info)
                    imu_result['effective_min_samples'] = effective_min_samples
                    imu_results[phase] = imu_result
                    
                    # Save with pretty formatting for human readability
                    output_file = output_dir / 'imu' / f'{phase}.json'
                    with open(output_file, 'w') as f:
                        json.dump(imu_result, f, indent=2, sort_keys=True)
                    
                    # Show quality warnings
                    quality = imu_result.get('quality_checks', {})
                    warnings = quality.get('warnings', [])
                    warning_str = f" (⚠️ {len(warnings)} warnings)" if warnings else ""
                    
                    print(f"✓ IMU {phase}: {len(filtered_imu.timestamps)} samples, rate: {imu_result['metadata']['rate_est_hz']:.1f} Hz{warning_str}")
                    for warning in warnings:
                        print(f"  ⚠️ {warning}")
                else:
                    print(f"✗ IMU {phase}: insufficient samples ({len(filtered_imu.timestamps)} < {effective_min_samples})")
            else:
                print(f"✗ IMU {phase}: no data found")
            
            # Process odometry data
            odom_data = reader.read_odom_data(args.odom_topic)
            if odom_data and len(odom_data.timestamps) > 0:
                filtered_odom, segmentation_info = calculator.filter_data_by_window(
                    odom_data, args.segment_mode, args.wz_min, args.vx_min
                )
                
                # Check if data was rejected due to min_duration
                if segmentation_info.get('rejected', False):
                    print(f"✗ Odom {phase}: {segmentation_info['rejected_reason']}")
                elif len(filtered_odom.timestamps) >= effective_min_samples:
                    odom_result = calculator.calculate_odom_covariances(filtered_odom, segmentation_info)
                    odom_result['effective_min_samples'] = effective_min_samples
                    odom_results[phase] = odom_result
                    
                    # Save with pretty formatting for human readability
                    output_file = output_dir / 'odom' / f'{phase}.json'
                    with open(output_file, 'w') as f:
                        json.dump(odom_result, f, indent=2, sort_keys=True)
                    
                    # Show quality warnings
                    quality = odom_result.get('quality_checks', {})
                    warnings = quality.get('warnings', [])
                    warning_str = f" (⚠️ {len(warnings)} warnings)" if warnings else ""
                    
                    print(f"✓ Odom {phase}: {len(filtered_odom.timestamps)} samples, rate: {odom_result['metadata']['rate_est_hz']:.1f} Hz{warning_str}")
                    for warning in warnings:
                        print(f"  ⚠️ {warning}")
                else:
                    print(f"✗ Odom {phase}: insufficient samples ({len(filtered_odom.timestamps)} < {effective_min_samples})")
            else:
                print(f"✗ Odom {phase}: no data found")
        
        # Generate summary and reports
        if imu_results or odom_results:
            generate_summary_report(imu_results, odom_results, output_dir)
            print(f"\n✓ Analysis complete. Results saved to: {output_dir}")
        else:
            print("✗ No valid results generated")
            sys.exit(1)
            
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
