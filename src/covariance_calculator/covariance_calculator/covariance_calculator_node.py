#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from typing import Dict, List
from .data_analyzer import DataAnalyzer, MotionSegment
from .imu_covariance_analyzer import IMUCovarianceAnalyzer
from .odom_covariance_analyzer import OdomCovarianceAnalyzer


class CovarianceCalculatorNode(Node):
    """Main node for calculating sensor covariance matrices from bag data."""
    
    def __init__(self):
        super().__init__('covariance_calculator')
        
        # Declare parameters
        self.declare_parameter('config_file', 'config/analysis_config.yaml')
        self.declare_parameter('data_directory', 'data/my_bags_outdoor_25')
        self.declare_parameter('output_directory', 'results')
        
        # Load configuration
        self.config = self._load_config()
        
        # Initialize analyzers
        self.data_analyzer = DataAnalyzer(self.config)
        self.imu_analyzer = IMUCovarianceAnalyzer(self.config)
        self.odom_analyzer = OdomCovarianceAnalyzer(self.config)
        
        # Results storage
        self.imu_results = {}
        self.odom_results = {}
        
        self.get_logger().info("Covariance Calculator Node initialized")
        
    def _load_config(self) -> Dict:
        """Load configuration from YAML file."""
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        
        try:
            # Try to load from package share directory first
            from ament_index_python.packages import get_package_share_directory
            package_share = get_package_share_directory('covariance_calculator')
            config_path = os.path.join(package_share, config_file)
            
            if not os.path.exists(config_path):
                # Fallback to relative path
                config_path = config_file
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                
            # Extract the nested configuration
            if 'covariance_calculator' in config:
                config = config['covariance_calculator']['ros__parameters']
            
            self.get_logger().info(f"Loaded configuration from {config_path}")
            return config
            
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            # Return default configuration
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict:
        """Get default configuration."""
        return {
            'data_directory': 'data/my_bags_outdoor_25',
            'stationary_bag': 'stationary_recording',
            'spin_bag': 'slow_spin_recording',
            'straight_bag': 'slow_straight_recording',
            'imu_topic': '/hardware_layer/imu/data_raw',
            'odom_topic': '/hardware_layer/diff_cont/odom',
            'joint_states_topic': '/hardware_layer/joint_states',
            'stationary_detection': {
                'wheel_velocity_threshold': 0.01,
                'min_stationary_duration': 2.0
            },
            'imu_analysis': {
                'target_variables': ['yaw', 'yaw_rate'],
                'filter_outliers': True,
                'outlier_std_threshold': 3.0
            },
            'odom_analysis': {
                'target_variables': ['velocity_x', 'velocity_y'],
                'filter_outliers': True,
                'outlier_std_threshold': 3.0
            },
            'output_directory': 'results'
        }
    
    def analyze_bag_data(self, bag_name: str, motion_type: str) -> bool:
        """
        Analyze a single bag file.
        
        Args:
            bag_name: Name of the bag directory
            motion_type: Type of motion ('stationary', 'spinning', 'straight')
            
        Returns:
            True if analysis was successful
        """
        data_dir = self.get_parameter('data_directory').get_parameter_value().string_value
        bag_path = os.path.join(data_dir, bag_name)
        
        if not os.path.exists(bag_path):
            self.get_logger().error(f"Bag directory not found: {bag_path}")
            return False
        
        self.get_logger().info(f"Analyzing {motion_type} data from {bag_path}")
        
        # Load bag data
        if not self.data_analyzer.load_bag_data(bag_path):
            self.get_logger().error(f"Failed to load bag data from {bag_path}")
            return False
        
        # Detect stationary periods (for stationary bags)
        if motion_type == 'stationary':
            stationary_segments = self.data_analyzer.detect_stationary_periods()
            if not stationary_segments:
                self.get_logger().warning("No stationary periods detected")
                return False
            
            # Use the longest stationary period
            segment = max(stationary_segments, key=lambda s: s.duration)
            self.get_logger().info(f"Using stationary period: {segment.duration:.2f}s")
        else:
            # For spinning/straight motion, analyze the entire bag
            if len(self.data_analyzer.imu_data) == 0:
                self.get_logger().error("No IMU data found")
                return False
            
            start_time = min(self.data_analyzer.imu_data.timestamps[0],
                           self.data_analyzer.odom_data.timestamps[0])
            end_time = max(self.data_analyzer.imu_data.timestamps[-1],
                         self.data_analyzer.odom_data.timestamps[-1])
            
            segment = MotionSegment(
                start_time=start_time,
                end_time=end_time,
                motion_type=motion_type,
                duration=end_time - start_time
            )
        
        # Analyze IMU data
        try:
            yaw_data, yaw_rate_data = self.data_analyzer.extract_imu_data_for_segment(segment)
            if len(yaw_data) > 0:
                imu_result = self.imu_analyzer.analyze_motion_segment(segment, yaw_data, yaw_rate_data)
                self.imu_results[motion_type] = imu_result
            else:
                self.get_logger().warning(f"No IMU data found for {motion_type} segment")
        except Exception as e:
            self.get_logger().error(f"IMU analysis failed: {e}")
        
        # Analyze Odometry data
        try:
            vx_data, vy_data = self.data_analyzer.extract_odom_data_for_segment(segment)
            if len(vx_data) > 0:
                odom_result = self.odom_analyzer.analyze_motion_segment(segment, vx_data, vy_data)
                self.odom_results[motion_type] = odom_result
            else:
                self.get_logger().warning(f"No Odometry data found for {motion_type} segment")
        except Exception as e:
            self.get_logger().error(f"Odometry analysis failed: {e}")
        
        return True
    
    def analyze_all_bags(self) -> None:
        """Analyze all configured bag files."""
        bags_to_analyze = [
            (self.config.get('stationary_bag', 'stationary_recording'), 'stationary'),
            (self.config.get('spin_bag', 'slow_spin_recording'), 'spinning'),
            (self.config.get('straight_bag', 'slow_straight_recording'), 'straight')
        ]
        
        successful_analyses = 0
        
        for bag_name, motion_type in bags_to_analyze:
            if self.analyze_bag_data(bag_name, motion_type):
                successful_analyses += 1
            else:
                self.get_logger().warning(f"Failed to analyze {motion_type} bag: {bag_name}")
        
        self.get_logger().info(f"Completed analysis of {successful_analyses}/{len(bags_to_analyze)} bags")
        
        # Generate summary and outputs
        if successful_analyses > 0:
            self._generate_outputs()
    
    def _generate_outputs(self) -> None:
        """Generate analysis outputs and robot_localization configuration."""
        output_dir = self.get_parameter('output_directory').get_parameter_value().string_value
        os.makedirs(output_dir, exist_ok=True)
        
        # Save individual results
        if self.imu_results:
            imu_output_path = os.path.join(output_dir, 'imu_covariance_results.txt')
            self.imu_analyzer.results = self.imu_results
            self.imu_analyzer.save_results_to_file(imu_output_path)
        
        if self.odom_results:
            odom_output_path = os.path.join(output_dir, 'odom_covariance_results.txt')
            self.odom_analyzer.results = self.odom_results
            self.odom_analyzer.save_results_to_file(odom_output_path)
        
        # Generate robot_localization configuration
        self._generate_robot_localization_config(output_dir)
        
        # Generate summary report
        self._generate_summary_report(output_dir)
    
    def _generate_robot_localization_config(self, output_dir: str) -> None:
        """Generate robot_localization YAML configuration."""
        config_output = {}
        
        # Add IMU configuration
        if self.imu_results:
            self.imu_analyzer.results = self.imu_results
            imu_config = self.imu_analyzer.generate_robot_localization_config()
            config_output.update(imu_config)
        
        # Add Odometry configuration
        if self.odom_results:
            self.odom_analyzer.results = self.odom_results
            odom_config = self.odom_analyzer.generate_robot_localization_config()
            config_output.update(odom_config)
        
        # Save configuration
        if config_output:
            config_path = os.path.join(output_dir, 'robot_localization_config.yaml')
            try:
                with open(config_path, 'w') as f:
                    yaml.dump(config_output, f, default_flow_style=False, indent=2)
                self.get_logger().info(f"Robot localization config saved to {config_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to save robot localization config: {e}")
    
    def _generate_summary_report(self, output_dir: str) -> None:
        """Generate a comprehensive summary report."""
        summary_path = os.path.join(output_dir, 'covariance_analysis_summary.txt')
        
        try:
            with open(summary_path, 'w') as f:
                f.write("Covariance Analysis Summary Report\n")
                f.write("=" * 50 + "\n\n")
                
                # IMU Summary
                if self.imu_results:
                    f.write("IMU Analysis Summary:\n")
                    f.write("-" * 30 + "\n")
                    for motion_type, result in self.imu_results.items():
                        f.write(f"{motion_type.capitalize()}:\n")
                        f.write(f"  Yaw std dev: {result.yaw_variance**0.5:.6f} rad ({(result.yaw_variance**0.5)*180/3.14159:.4f}°)\n")
                        f.write(f"  Yaw rate std dev: {result.yaw_rate_variance**0.5:.6f} rad/s\n")
                        f.write(f"  Samples: {result.sample_count}\n\n")
                
                # Odometry Summary
                if self.odom_results:
                    f.write("Odometry Analysis Summary:\n")
                    f.write("-" * 30 + "\n")
                    for motion_type, result in self.odom_results.items():
                        f.write(f"{motion_type.capitalize()}:\n")
                        f.write(f"  Velocity X std dev: {result.velocity_x_variance**0.5:.6f} m/s\n")
                        f.write(f"  Velocity Y std dev: {result.velocity_y_variance**0.5:.6f} m/s\n")
                        f.write(f"  Samples: {result.sample_count}\n\n")
                
                # Recommendations
                f.write("Recommendations for robot_localization:\n")
                f.write("-" * 40 + "\n")
                
                if 'stationary' in self.imu_results:
                    result = self.imu_results['stationary']
                    f.write(f"IMU Covariance (conservative, from stationary data):\n")
                    f.write(f"  imu0_covariance diagonal values:\n")
                    f.write(f"    Position 35 (yaw): {result.yaw_variance:.8f}\n")
                    f.write(f"    Position 41 (yaw_rate): {result.yaw_rate_variance:.8f}\n\n")
                
                if 'stationary' in self.odom_results:
                    result = self.odom_results['stationary']
                    f.write(f"Odometry Covariance (conservative, from stationary data):\n")
                    f.write(f"  odom0_covariance diagonal values:\n")
                    f.write(f"    Position 6 (vel_x): {result.velocity_x_variance:.8f}\n")
                    f.write(f"    Position 7 (vel_y): {result.velocity_y_variance:.8f}\n\n")
            
            self.get_logger().info(f"Summary report saved to {summary_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to generate summary report: {e}")
    
    def run_analysis(self) -> None:
        """Run the complete covariance analysis."""
        self.get_logger().info("Starting covariance analysis...")
        
        try:
            self.analyze_all_bags()
            self.get_logger().info("Covariance analysis completed successfully")
            
            # Print quick summary
            if self.imu_results or self.odom_results:
                print("\n" + "="*60)
                print("COVARIANCE ANALYSIS COMPLETED")
                print("="*60)
                
                if self.imu_results:
                    print(f"IMU results: {len(self.imu_results)} motion types analyzed")
                    
                if self.odom_results:
                    print(f"Odometry results: {len(self.odom_results)} motion types analyzed")
                
                output_dir = self.get_parameter('output_directory').get_parameter_value().string_value
                print(f"Results saved to: {output_dir}")
                print("Check robot_localization_config.yaml for ready-to-use configuration")
                print("="*60)
            
        except Exception as e:
            self.get_logger().error(f"Analysis failed: {e}")
            raise


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = CovarianceCalculatorNode()
        
        # Run analysis
        node.run_analysis()
        
        # Keep node alive briefly to ensure all logging is output
        rclpy.spin_once(node, timeout_sec=1.0)
        
    except KeyboardInterrupt:
        print("Analysis interrupted by user")
    except Exception as e:
        print(f"Analysis failed: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
