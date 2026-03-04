#!/usr/bin/env python3

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from .utils import (calculate_sample_variance, calculate_sample_covariance, 
                   create_covariance_matrix_2x2, format_covariance_for_robot_localization,
                   print_statistics)
from .data_analyzer import MotionSegment


@dataclass
class IMUCovarianceResult:
    """Container for IMU covariance analysis results."""
    motion_type: str
    yaw_variance: float
    yaw_rate_variance: float
    yaw_yaw_rate_covariance: float
    covariance_matrix: np.ndarray
    sample_count: int
    outliers_removed: int
    
    def get_robot_localization_format(self) -> List[float]:
        """Get covariance in robot_localization 6x6 format."""
        # IMU yaw is index 5, yaw_rate is index 11 in robot_localization
        return format_covariance_for_robot_localization(self.covariance_matrix, [5, 11])


class IMUCovarianceAnalyzer:
    """Analyzes IMU data to calculate yaw and yaw_rate covariance matrices."""
    
    def __init__(self, config: Dict):
        self.config = config
        self.imu_config = config.get('imu_analysis', {})
        self.filter_outliers = self.imu_config.get('filter_outliers', True)
        self.outlier_threshold = self.imu_config.get('outlier_std_threshold', 3.0)
        
        # Results storage
        self.results = {}
    
    def analyze_motion_segment(self, segment: MotionSegment, yaw_data: np.ndarray, 
                             yaw_rate_data: np.ndarray) -> IMUCovarianceResult:
        """
        Analyze IMU data for a specific motion segment.
        
        Args:
            segment: Motion segment information
            yaw_data: Yaw angle data in radians
            yaw_rate_data: Yaw rate data in rad/s
            
        Returns:
            IMU covariance analysis result
        """
        if len(yaw_data) != len(yaw_rate_data):
            raise ValueError("Yaw and yaw_rate data must have the same length")
        
        if len(yaw_data) < 3:
            print(f"Warning: Insufficient data for {segment.motion_type} analysis ({len(yaw_data)} samples)")
            return self._create_empty_result(segment.motion_type)
        
        print(f"\nAnalyzing IMU data for {segment.motion_type} motion:")
        print(f"  Duration: {segment.duration:.2f}s")
        print(f"  Sample count: {len(yaw_data)}")
        
        # Store original data for statistics
        original_yaw = yaw_data.copy()
        original_yaw_rate = yaw_rate_data.copy()
        
        # Handle yaw angle wrapping for stationary analysis
        if segment.motion_type == 'stationary':
            yaw_data = self._unwrap_yaw_angles(yaw_data)
        
        # Remove outliers if enabled
        outliers_removed = 0
        if self.filter_outliers and len(yaw_data) > 10:
            from .data_analyzer import DataAnalyzer
            analyzer = DataAnalyzer(self.config)
            filtered_yaw, filtered_yaw_rate = analyzer.filter_outliers_from_data(
                yaw_data, yaw_rate_data, self.outlier_threshold
            )
            outliers_removed = len(yaw_data) - len(filtered_yaw)
            yaw_data, yaw_rate_data = filtered_yaw, filtered_yaw_rate
        
        if len(yaw_data) < 3:
            print(f"Warning: Insufficient data after outlier removal ({len(yaw_data)} samples)")
            return self._create_empty_result(segment.motion_type)
        
        # Print statistics
        print_statistics(original_yaw, "Original Yaw")
        print_statistics(original_yaw_rate, "Original Yaw Rate")
        if outliers_removed > 0:
            print_statistics(yaw_data, "Filtered Yaw")
            print_statistics(yaw_rate_data, "Filtered Yaw Rate")
        
        # Calculate variances and covariance
        yaw_variance = calculate_sample_variance(yaw_data)
        yaw_rate_variance = calculate_sample_variance(yaw_rate_data)
        yaw_yaw_rate_covariance = calculate_sample_covariance(yaw_data, yaw_rate_data)
        
        # Create covariance matrix
        cov_matrix = create_covariance_matrix_2x2(
            yaw_variance, yaw_rate_variance, yaw_yaw_rate_covariance
        )
        
        # Create result
        result = IMUCovarianceResult(
            motion_type=segment.motion_type,
            yaw_variance=yaw_variance,
            yaw_rate_variance=yaw_rate_variance,
            yaw_yaw_rate_covariance=yaw_yaw_rate_covariance,
            covariance_matrix=cov_matrix,
            sample_count=len(yaw_data),
            outliers_removed=outliers_removed
        )
        
        # Store result
        self.results[segment.motion_type] = result
        
        # Print results
        self._print_analysis_results(result)
        
        return result
    
    def _unwrap_yaw_angles(self, yaw_data: np.ndarray) -> np.ndarray:
        """
        Unwrap yaw angles to handle 2π discontinuities.
        
        Args:
            yaw_data: Raw yaw angle data
            
        Returns:
            Unwrapped yaw angles
        """
        return np.unwrap(yaw_data)
    
    def _create_empty_result(self, motion_type: str) -> IMUCovarianceResult:
        """Create an empty result for insufficient data cases."""
        return IMUCovarianceResult(
            motion_type=motion_type,
            yaw_variance=0.0,
            yaw_rate_variance=0.0,
            yaw_yaw_rate_covariance=0.0,
            covariance_matrix=np.zeros((2, 2)),
            sample_count=0,
            outliers_removed=0
        )
    
    def _print_analysis_results(self, result: IMUCovarianceResult) -> None:
        """Print analysis results in a formatted way."""
        print(f"\nIMU Covariance Analysis Results ({result.motion_type}):")
        print(f"  Sample count: {result.sample_count}")
        if result.outliers_removed > 0:
            print(f"  Outliers removed: {result.outliers_removed}")
        print(f"  Yaw variance: {result.yaw_variance:.8f} rad²")
        print(f"  Yaw rate variance: {result.yaw_rate_variance:.8f} (rad/s)²")
        print(f"  Yaw-YawRate covariance: {result.yaw_yaw_rate_covariance:.8f} rad·(rad/s)")
        print(f"  Yaw std dev: {np.sqrt(result.yaw_variance):.6f} rad ({np.degrees(np.sqrt(result.yaw_variance)):.4f}°)")
        print(f"  Yaw rate std dev: {np.sqrt(result.yaw_rate_variance):.6f} rad/s ({np.degrees(np.sqrt(result.yaw_rate_variance)):.4f}°/s)")
        
        print(f"  Covariance Matrix:")
        print(f"    [{result.covariance_matrix[0,0]:.8f}  {result.covariance_matrix[0,1]:.8f}]")
        print(f"    [{result.covariance_matrix[1,0]:.8f}  {result.covariance_matrix[1,1]:.8f}]")
    
    def analyze_all_motion_types(self, data_analyzer, motion_segments: List[MotionSegment]) -> Dict[str, IMUCovarianceResult]:
        """
        Analyze IMU data for all motion segments.
        
        Args:
            data_analyzer: DataAnalyzer instance with loaded data
            motion_segments: List of motion segments to analyze
            
        Returns:
            Dictionary of results by motion type
        """
        results = {}
        
        for segment in motion_segments:
            try:
                # Extract IMU data for this segment
                yaw_data, yaw_rate_data = data_analyzer.extract_imu_data_for_segment(segment)
                
                if len(yaw_data) == 0:
                    print(f"No IMU data found for {segment.motion_type} segment")
                    continue
                
                # Analyze the segment
                result = self.analyze_motion_segment(segment, yaw_data, yaw_rate_data)
                results[segment.motion_type] = result
                
            except Exception as e:
                print(f"Error analyzing {segment.motion_type} segment: {e}")
                continue
        
        return results
    
    def get_recommended_covariance(self, motion_type: str = 'stationary') -> Optional[IMUCovarianceResult]:
        """
        Get recommended covariance matrix for robot_localization.
        
        Args:
            motion_type: Motion type to use for recommendation ('stationary' is most conservative)
            
        Returns:
            Recommended covariance result, or None if not available
        """
        if motion_type in self.results:
            return self.results[motion_type]
        
        # Fallback to any available result
        if self.results:
            return list(self.results.values())[0]
        
        return None
    
    def generate_robot_localization_config(self) -> Dict[str, any]:
        """
        Generate robot_localization configuration for IMU.
        
        Returns:
            Dictionary with robot_localization IMU configuration
        """
        recommended = self.get_recommended_covariance('stationary')
        
        if recommended is None:
            print("No IMU covariance data available for configuration generation")
            return {}
        
        # robot_localization IMU configuration
        # Only yaw (index 5) and yaw_rate (index 11) are used
        config = {
            'imu0': '/hardware_layer/imu/data_raw',
            'imu0_config': [False, False, False,    # x, y, z position (not used)
                           False, False, True,     # roll, pitch, yaw (only yaw)
                           False, False, False,    # x, y, z velocity (not used)
                           False, False, True,     # roll_rate, pitch_rate, yaw_rate (only yaw_rate)
                           False, False, False],   # x, y, z acceleration (not used)
            'imu0_differential': False,
            'imu0_relative': False,
            'imu0_queue_size': 10,
            'imu0_remove_gravitational_acceleration': True
        }
        
        # Create 6x6 covariance matrix (15x15 for robot_localization, but we use 6x6 format)
        covariance_6x6 = np.zeros((6, 6))
        covariance_6x6[5, 5] = recommended.yaw_variance  # yaw variance
        # Note: robot_localization uses different indexing for twist covariance
        # We'll handle this in the output formatting
        
        # Convert to list format expected by robot_localization
        config['imu0_covariance'] = covariance_6x6.flatten().tolist()
        
        return config
    
    def save_results_to_file(self, output_path: str) -> None:
        """
        Save analysis results to a file.
        
        Args:
            output_path: Path to save results
        """
        try:
            with open(output_path, 'w') as f:
                f.write("IMU Covariance Analysis Results\n")
                f.write("=" * 40 + "\n\n")
                
                for motion_type, result in self.results.items():
                    f.write(f"Motion Type: {motion_type.upper()}\n")
                    f.write(f"Sample Count: {result.sample_count}\n")
                    f.write(f"Outliers Removed: {result.outliers_removed}\n")
                    f.write(f"Yaw Variance: {result.yaw_variance:.8f} rad²\n")
                    f.write(f"Yaw Rate Variance: {result.yaw_rate_variance:.8f} (rad/s)²\n")
                    f.write(f"Yaw-YawRate Covariance: {result.yaw_yaw_rate_covariance:.8f}\n")
                    f.write(f"Yaw Std Dev: {np.sqrt(result.yaw_variance):.6f} rad ({np.degrees(np.sqrt(result.yaw_variance)):.4f}°)\n")
                    f.write(f"Yaw Rate Std Dev: {np.sqrt(result.yaw_rate_variance):.6f} rad/s ({np.degrees(np.sqrt(result.yaw_rate_variance)):.4f}°/s)\n")
                    f.write("Covariance Matrix:\n")
                    f.write(f"  [{result.covariance_matrix[0,0]:.8f}  {result.covariance_matrix[0,1]:.8f}]\n")
                    f.write(f"  [{result.covariance_matrix[1,0]:.8f}  {result.covariance_matrix[1,1]:.8f}]\n")
                    f.write("\n" + "-" * 40 + "\n\n")
                
                # Add robot_localization configuration
                config = self.generate_robot_localization_config()
                if config:
                    f.write("Robot Localization Configuration:\n")
                    f.write("-" * 40 + "\n")
                    for key, value in config.items():
                        f.write(f"{key}: {value}\n")
            
            print(f"Results saved to {output_path}")
            
        except Exception as e:
            print(f"Error saving results: {e}")
