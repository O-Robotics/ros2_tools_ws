#!/usr/bin/env python3

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from .utils import (calculate_sample_variance, calculate_sample_covariance, 
                   create_covariance_matrix_2x2, format_covariance_for_robot_localization,
                   print_statistics)
from .data_analyzer import MotionSegment


@dataclass
class OdomCovarianceResult:
    """Container for Odometry covariance analysis results."""
    motion_type: str
    velocity_x_variance: float
    velocity_y_variance: float
    velocity_x_y_covariance: float
    covariance_matrix: np.ndarray
    sample_count: int
    outliers_removed: int
    
    def get_robot_localization_format(self) -> List[float]:
        """Get covariance in robot_localization 6x6 format."""
        # Odometry velocity_x is index 6, velocity_y is index 7 in robot_localization
        return format_covariance_for_robot_localization(self.covariance_matrix, [6, 7])


class OdomCovarianceAnalyzer:
    """Analyzes Odometry data to calculate velocity_x and velocity_y covariance matrices."""
    
    def __init__(self, config: Dict):
        self.config = config
        self.odom_config = config.get('odom_analysis', {})
        self.filter_outliers = self.odom_config.get('filter_outliers', True)
        self.outlier_threshold = self.odom_config.get('outlier_std_threshold', 3.0)
        
        # Results storage
        self.results = {}
    
    def analyze_motion_segment(self, segment: MotionSegment, velocity_x_data: np.ndarray, 
                             velocity_y_data: np.ndarray) -> OdomCovarianceResult:
        """
        Analyze Odometry data for a specific motion segment.
        
        Args:
            segment: Motion segment information
            velocity_x_data: Linear velocity x data in m/s
            velocity_y_data: Linear velocity y data in m/s
            
        Returns:
            Odometry covariance analysis result
        """
        if len(velocity_x_data) != len(velocity_y_data):
            raise ValueError("Velocity x and y data must have the same length")
        
        if len(velocity_x_data) < 3:
            print(f"Warning: Insufficient data for {segment.motion_type} analysis ({len(velocity_x_data)} samples)")
            return self._create_empty_result(segment.motion_type)
        
        print(f"\nAnalyzing Odometry data for {segment.motion_type} motion:")
        print(f"  Duration: {segment.duration:.2f}s")
        print(f"  Sample count: {len(velocity_x_data)}")
        
        # Store original data for statistics
        original_vx = velocity_x_data.copy()
        original_vy = velocity_y_data.copy()
        
        # Remove outliers if enabled
        outliers_removed = 0
        if self.filter_outliers and len(velocity_x_data) > 10:
            from .data_analyzer import DataAnalyzer
            analyzer = DataAnalyzer(self.config)
            filtered_vx, filtered_vy = analyzer.filter_outliers_from_data(
                velocity_x_data, velocity_y_data, self.outlier_threshold
            )
            outliers_removed = len(velocity_x_data) - len(filtered_vx)
            velocity_x_data, velocity_y_data = filtered_vx, filtered_vy
        
        if len(velocity_x_data) < 3:
            print(f"Warning: Insufficient data after outlier removal ({len(velocity_x_data)} samples)")
            return self._create_empty_result(segment.motion_type)
        
        # Print statistics
        print_statistics(original_vx, "Original Velocity X")
        print_statistics(original_vy, "Original Velocity Y")
        if outliers_removed > 0:
            print_statistics(velocity_x_data, "Filtered Velocity X")
            print_statistics(velocity_y_data, "Filtered Velocity Y")
        
        # Calculate variances and covariance
        vx_variance = calculate_sample_variance(velocity_x_data)
        vy_variance = calculate_sample_variance(velocity_y_data)
        vx_vy_covariance = calculate_sample_covariance(velocity_x_data, velocity_y_data)
        
        # Create covariance matrix
        cov_matrix = create_covariance_matrix_2x2(
            vx_variance, vy_variance, vx_vy_covariance
        )
        
        # Create result
        result = OdomCovarianceResult(
            motion_type=segment.motion_type,
            velocity_x_variance=vx_variance,
            velocity_y_variance=vy_variance,
            velocity_x_y_covariance=vx_vy_covariance,
            covariance_matrix=cov_matrix,
            sample_count=len(velocity_x_data),
            outliers_removed=outliers_removed
        )
        
        # Store result
        self.results[segment.motion_type] = result
        
        # Print results
        self._print_analysis_results(result)
        
        return result
    
    def _create_empty_result(self, motion_type: str) -> OdomCovarianceResult:
        """Create an empty result for insufficient data cases."""
        return OdomCovarianceResult(
            motion_type=motion_type,
            velocity_x_variance=0.0,
            velocity_y_variance=0.0,
            velocity_x_y_covariance=0.0,
            covariance_matrix=np.zeros((2, 2)),
            sample_count=0,
            outliers_removed=0
        )
    
    def _print_analysis_results(self, result: OdomCovarianceResult) -> None:
        """Print analysis results in a formatted way."""
        print(f"\nOdometry Covariance Analysis Results ({result.motion_type}):")
        print(f"  Sample count: {result.sample_count}")
        if result.outliers_removed > 0:
            print(f"  Outliers removed: {result.outliers_removed}")
        print(f"  Velocity X variance: {result.velocity_x_variance:.8f} (m/s)²")
        print(f"  Velocity Y variance: {result.velocity_y_variance:.8f} (m/s)²")
        print(f"  Velocity X-Y covariance: {result.velocity_x_y_covariance:.8f} (m/s)²")
        print(f"  Velocity X std dev: {np.sqrt(result.velocity_x_variance):.6f} m/s")
        print(f"  Velocity Y std dev: {np.sqrt(result.velocity_y_variance):.6f} m/s")
        
        print(f"  Covariance Matrix:")
        print(f"    [{result.covariance_matrix[0,0]:.8f}  {result.covariance_matrix[0,1]:.8f}]")
        print(f"    [{result.covariance_matrix[1,0]:.8f}  {result.covariance_matrix[1,1]:.8f}]")
        
        # Motion-specific insights
        self._print_motion_insights(result)
    
    def _print_motion_insights(self, result: OdomCovarianceResult) -> None:
        """Print motion-specific insights."""
        vx_std = np.sqrt(result.velocity_x_variance)
        vy_std = np.sqrt(result.velocity_y_variance)
        
        print(f"  Motion Insights:")
        if result.motion_type == 'stationary':
            print(f"    - Forward/backward drift: {vx_std:.6f} m/s")
            print(f"    - Lateral drift: {vy_std:.6f} m/s")
            if vx_std > 0.01:
                print(f"    - WARNING: High forward drift detected!")
            if vy_std > 0.01:
                print(f"    - WARNING: High lateral drift detected!")
        
        elif result.motion_type == 'spinning':
            print(f"    - Forward velocity noise during spin: {vx_std:.6f} m/s")
            print(f"    - Lateral velocity noise during spin: {vy_std:.6f} m/s")
            if vx_std > 0.05:
                print(f"    - WARNING: High forward velocity noise during rotation!")
        
        elif result.motion_type == 'straight':
            print(f"    - Forward velocity accuracy: {vx_std:.6f} m/s")
            print(f"    - Lateral drift during straight motion: {vy_std:.6f} m/s")
            if vy_std > 0.02:
                print(f"    - WARNING: High lateral drift during straight motion!")
    
    def analyze_all_motion_types(self, data_analyzer, motion_segments: List[MotionSegment]) -> Dict[str, OdomCovarianceResult]:
        """
        Analyze Odometry data for all motion segments.
        
        Args:
            data_analyzer: DataAnalyzer instance with loaded data
            motion_segments: List of motion segments to analyze
            
        Returns:
            Dictionary of results by motion type
        """
        results = {}
        
        for segment in motion_segments:
            try:
                # Extract Odometry data for this segment
                vx_data, vy_data = data_analyzer.extract_odom_data_for_segment(segment)
                
                if len(vx_data) == 0:
                    print(f"No Odometry data found for {segment.motion_type} segment")
                    continue
                
                # Analyze the segment
                result = self.analyze_motion_segment(segment, vx_data, vy_data)
                results[segment.motion_type] = result
                
            except Exception as e:
                print(f"Error analyzing {segment.motion_type} segment: {e}")
                continue
        
        return results
    
    def get_recommended_covariance(self, motion_type: str = 'stationary') -> Optional[OdomCovarianceResult]:
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
        Generate robot_localization configuration for Odometry.
        
        Returns:
            Dictionary with robot_localization Odometry configuration
        """
        recommended = self.get_recommended_covariance('stationary')
        
        if recommended is None:
            print("No Odometry covariance data available for configuration generation")
            return {}
        
        # robot_localization Odometry configuration
        # Only velocity_x (index 6) and velocity_y (index 7) are used
        config = {
            'odom0': '/hardware_layer/diff_cont/odom',
            'odom0_config': [False, False, False,    # x, y, z position (not used for velocity-only)
                            False, False, False,    # roll, pitch, yaw (not used for velocity-only)
                            True,  True,  False,    # x_vel, y_vel, z_vel (only x and y)
                            False, False, False,    # roll_rate, pitch_rate, yaw_rate (not used)
                            False, False, False],   # x_acc, y_acc, z_acc (not used)
            'odom0_differential': False,
            'odom0_relative': False,
            'odom0_queue_size': 10
        }
        
        # Create 6x6 covariance matrix for twist (velocity) part
        # In robot_localization, the twist covariance is separate from pose covariance
        covariance_6x6 = np.zeros((6, 6))
        covariance_6x6[0, 0] = recommended.velocity_x_variance  # x velocity variance
        covariance_6x6[1, 1] = recommended.velocity_y_variance  # y velocity variance
        covariance_6x6[0, 1] = recommended.velocity_x_y_covariance  # x-y velocity covariance
        covariance_6x6[1, 0] = recommended.velocity_x_y_covariance  # y-x velocity covariance
        
        # Convert to list format expected by robot_localization
        config['odom0_covariance'] = covariance_6x6.flatten().tolist()
        
        return config
    
    def compare_motion_types(self) -> None:
        """Compare covariance results across different motion types."""
        if len(self.results) < 2:
            print("Need at least 2 motion types for comparison")
            return
        
        print("\nMotion Type Comparison:")
        print("=" * 50)
        
        motion_types = list(self.results.keys())
        
        print(f"{'Motion Type':<15} {'Vel X Std':<12} {'Vel Y Std':<12} {'Samples':<10}")
        print("-" * 50)
        
        for motion_type in motion_types:
            result = self.results[motion_type]
            vx_std = np.sqrt(result.velocity_x_variance)
            vy_std = np.sqrt(result.velocity_y_variance)
            print(f"{motion_type:<15} {vx_std:<12.6f} {vy_std:<12.6f} {result.sample_count:<10}")
        
        # Recommendations
        print("\nRecommendations:")
        stationary_result = self.results.get('stationary')
        if stationary_result:
            vx_std = np.sqrt(stationary_result.velocity_x_variance)
            vy_std = np.sqrt(stationary_result.velocity_y_variance)
            print(f"- Use stationary covariance for conservative estimation:")
            print(f"  Velocity X variance: {stationary_result.velocity_x_variance:.8f}")
            print(f"  Velocity Y variance: {stationary_result.velocity_y_variance:.8f}")
            
            if vx_std > 0.01 or vy_std > 0.01:
                print(f"- WARNING: High velocity noise detected. Consider:")
                print(f"  * Checking wheel encoder calibration")
                print(f"  * Verifying differential drive parameters")
                print(f"  * Inspecting for mechanical issues")
    
    def save_results_to_file(self, output_path: str) -> None:
        """
        Save analysis results to a file.
        
        Args:
            output_path: Path to save results
        """
        try:
            with open(output_path, 'w') as f:
                f.write("Odometry Covariance Analysis Results\n")
                f.write("=" * 40 + "\n\n")
                
                for motion_type, result in self.results.items():
                    f.write(f"Motion Type: {motion_type.upper()}\n")
                    f.write(f"Sample Count: {result.sample_count}\n")
                    f.write(f"Outliers Removed: {result.outliers_removed}\n")
                    f.write(f"Velocity X Variance: {result.velocity_x_variance:.8f} (m/s)²\n")
                    f.write(f"Velocity Y Variance: {result.velocity_y_variance:.8f} (m/s)²\n")
                    f.write(f"Velocity X-Y Covariance: {result.velocity_x_y_covariance:.8f} (m/s)²\n")
                    f.write(f"Velocity X Std Dev: {np.sqrt(result.velocity_x_variance):.6f} m/s\n")
                    f.write(f"Velocity Y Std Dev: {np.sqrt(result.velocity_y_variance):.6f} m/s\n")
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
