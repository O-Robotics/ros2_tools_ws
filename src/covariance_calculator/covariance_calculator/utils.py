#!/usr/bin/env python3

import numpy as np
import math
from typing import List, Tuple, Dict, Any
from geometry_msgs.msg import Quaternion


def quaternion_to_yaw(quat: Quaternion) -> float:
    """
    Extract yaw angle from quaternion.
    
    Args:
        quat: Quaternion message
        
    Returns:
        Yaw angle in radians
    """
    # Convert quaternion to yaw using atan2
    # yaw = atan2(2(w*z + x*y), 1 - 2(y² + z²))
    yaw = math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    )
    return yaw


def calculate_sample_covariance(data1: np.ndarray, data2: np.ndarray) -> float:
    """
    Calculate sample covariance between two datasets.
    
    Args:
        data1: First dataset
        data2: Second dataset
        
    Returns:
        Sample covariance
    """
    if len(data1) != len(data2):
        raise ValueError("Datasets must have the same length")
    
    if len(data1) < 2:
        return 0.0
    
    mean1 = np.mean(data1)
    mean2 = np.mean(data2)
    
    covariance = np.sum((data1 - mean1) * (data2 - mean2)) / (len(data1) - 1)
    return covariance


def calculate_sample_variance(data: np.ndarray) -> float:
    """
    Calculate sample variance of a dataset.
    
    Args:
        data: Input dataset
        
    Returns:
        Sample variance
    """
    if len(data) < 2:
        return 0.0
    
    return np.var(data, ddof=1)  # ddof=1 for sample variance


def remove_outliers(data: np.ndarray, threshold: float = 3.0) -> Tuple[np.ndarray, np.ndarray]:
    """
    Remove statistical outliers using standard deviation method.
    
    Args:
        data: Input data array
        threshold: Number of standard deviations for outlier detection
        
    Returns:
        Tuple of (filtered_data, outlier_mask)
    """
    if len(data) < 3:
        return data, np.ones(len(data), dtype=bool)
    
    mean = np.mean(data)
    std = np.std(data, ddof=1)
    
    # Create mask for non-outliers
    outlier_mask = np.abs(data - mean) <= threshold * std
    
    return data[outlier_mask], outlier_mask


def create_covariance_matrix_2x2(var1: float, var2: float, cov12: float) -> np.ndarray:
    """
    Create a 2x2 covariance matrix.
    
    Args:
        var1: Variance of first variable
        var2: Variance of second variable
        cov12: Covariance between variables
        
    Returns:
        2x2 covariance matrix
    """
    return np.array([[var1, cov12],
                     [cov12, var2]])


def format_covariance_for_robot_localization(cov_matrix: np.ndarray, 
                                           variable_indices: List[int]) -> List[float]:
    """
    Format covariance matrix for robot_localization package (6x6 format).
    
    Args:
        cov_matrix: 2x2 covariance matrix
        variable_indices: Indices in the 6x6 matrix where values should be placed
        
    Returns:
        36-element list representing 6x6 covariance matrix
    """
    # Initialize 6x6 matrix with zeros
    full_cov = np.zeros((6, 6))
    
    # Place the 2x2 matrix at specified indices
    if len(variable_indices) == 2:
        i, j = variable_indices
        full_cov[i, i] = cov_matrix[0, 0]  # var1
        full_cov[j, j] = cov_matrix[1, 1]  # var2
        full_cov[i, j] = cov_matrix[0, 1]  # cov12
        full_cov[j, i] = cov_matrix[1, 0]  # cov21
    
    # Convert to flat list (row-major order)
    return full_cov.flatten().tolist()


def calculate_time_differences(timestamps: List[float]) -> np.ndarray:
    """
    Calculate time differences between consecutive timestamps.
    
    Args:
        timestamps: List of timestamps in seconds
        
    Returns:
        Array of time differences
    """
    if len(timestamps) < 2:
        return np.array([])
    
    timestamps = np.array(timestamps)
    return np.diff(timestamps)


def interpolate_data(timestamps: np.ndarray, values: np.ndarray, 
                    target_timestamps: np.ndarray) -> np.ndarray:
    """
    Interpolate data to target timestamps.
    
    Args:
        timestamps: Original timestamps
        values: Original values
        target_timestamps: Target timestamps for interpolation
        
    Returns:
        Interpolated values
    """
    return np.interp(target_timestamps, timestamps, values)


def validate_bag_data(data_dict: Dict[str, Any]) -> bool:
    """
    Validate that bag data contains required topics and sufficient data.
    
    Args:
        data_dict: Dictionary containing bag data
        
    Returns:
        True if data is valid, False otherwise
    """
    required_topics = ['imu', 'odom', 'joint_states']
    
    for topic in required_topics:
        if topic not in data_dict:
            print(f"Missing required topic: {topic}")
            return False
        
        if len(data_dict[topic]) < 10:  # Minimum 10 messages
            print(f"Insufficient data for topic {topic}: {len(data_dict[topic])} messages")
            return False
    
    return True


def print_statistics(data: np.ndarray, name: str) -> None:
    """
    Print basic statistics for a dataset.
    
    Args:
        data: Input data
        name: Name of the dataset for display
    """
    if len(data) == 0:
        print(f"{name}: No data")
        return
    
    print(f"{name} Statistics:")
    print(f"  Count: {len(data)}")
    print(f"  Mean: {np.mean(data):.6f}")
    print(f"  Std Dev: {np.std(data, ddof=1):.6f}")
    print(f"  Min: {np.min(data):.6f}")
    print(f"  Max: {np.max(data):.6f}")
    print(f"  Range: {np.max(data) - np.min(data):.6f}")
