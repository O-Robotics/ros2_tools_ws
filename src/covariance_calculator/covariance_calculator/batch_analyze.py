#!/usr/bin/env python3

import os
import sys
import argparse
from typing import List, Dict
from .covariance_calculator_node import CovarianceCalculatorNode
import rclpy


def main():
    """Batch analysis script for multiple bag datasets."""
    parser = argparse.ArgumentParser(description='Batch analyze covariance from multiple bag datasets')
    parser.add_argument('--data-dir', type=str, required=True,
                       help='Base directory containing bag datasets')
    parser.add_argument('--output-dir', type=str, default='batch_results',
                       help='Output directory for results')
    parser.add_argument('--config', type=str, default='config/analysis_config.yaml',
                       help='Configuration file path')
    parser.add_argument('--datasets', type=str, nargs='+',
                       help='Specific dataset names to analyze (default: all found)')
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Find all datasets if not specified
        if args.datasets is None:
            datasets = find_bag_datasets(args.data_dir)
        else:
            datasets = args.datasets
        
        if not datasets:
            print(f"No bag datasets found in {args.data_dir}")
            return 1
        
        print(f"Found {len(datasets)} datasets to analyze: {datasets}")
        
        # Analyze each dataset
        results = {}
        for dataset in datasets:
            print(f"\n{'='*60}")
            print(f"Analyzing dataset: {dataset}")
            print(f"{'='*60}")
            
            try:
                result = analyze_dataset(dataset, args.data_dir, args.output_dir, args.config)
                results[dataset] = result
                print(f"✓ Successfully analyzed {dataset}")
            except Exception as e:
                print(f"✗ Failed to analyze {dataset}: {e}")
                results[dataset] = None
        
        # Generate batch summary
        generate_batch_summary(results, args.output_dir)
        
        # Print final summary
        successful = sum(1 for r in results.values() if r is not None)
        print(f"\n{'='*60}")
        print(f"BATCH ANALYSIS COMPLETE")
        print(f"{'='*60}")
        print(f"Successful: {successful}/{len(datasets)}")
        print(f"Results saved to: {args.output_dir}")
        print(f"{'='*60}")
        
        return 0 if successful > 0 else 1
        
    except KeyboardInterrupt:
        print("\nBatch analysis interrupted by user")
        return 1
    except Exception as e:
        print(f"Batch analysis failed: {e}")
        return 1
    finally:
        rclpy.shutdown()


def find_bag_datasets(base_dir: str) -> List[str]:
    """Find all bag datasets in the base directory."""
    datasets = []
    
    if not os.path.exists(base_dir):
        return datasets
    
    for item in os.listdir(base_dir):
        item_path = os.path.join(base_dir, item)
        if os.path.isdir(item_path):
            # Check if it contains bag files or subdirectories with bag files
            if has_bag_files(item_path):
                datasets.append(item)
    
    return sorted(datasets)


def has_bag_files(directory: str) -> bool:
    """Check if directory contains bag files (directly or in subdirectories)."""
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.db3') or file.endswith('.db3.zstd'):
                return True
        # Also check for metadata.yaml which indicates a bag directory
        if 'metadata.yaml' in files:
            return True
    return False


def analyze_dataset(dataset_name: str, base_data_dir: str, base_output_dir: str, config_file: str) -> Dict:
    """Analyze a single dataset."""
    # Create dataset-specific output directory
    dataset_output_dir = os.path.join(base_output_dir, dataset_name)
    os.makedirs(dataset_output_dir, exist_ok=True)
    
    # Create node with dataset-specific parameters
    node = CovarianceCalculatorNode()
    
    # Override parameters for this dataset
    node.set_parameters([
        rclpy.parameter.Parameter('data_directory', rclpy.Parameter.Type.STRING, 
                                os.path.join(base_data_dir, dataset_name)),
        rclpy.parameter.Parameter('output_directory', rclpy.Parameter.Type.STRING, 
                                dataset_output_dir),
        rclpy.parameter.Parameter('config_file', rclpy.Parameter.Type.STRING, 
                                config_file)
    ])
    
    # Run analysis
    node.run_analysis()
    
    # Collect results
    results = {
        'dataset': dataset_name,
        'imu_results': dict(node.imu_results),
        'odom_results': dict(node.odom_results),
        'output_dir': dataset_output_dir
    }
    
    return results


def generate_batch_summary(results: Dict, output_dir: str) -> None:
    """Generate a summary report for all analyzed datasets."""
    summary_path = os.path.join(output_dir, 'batch_analysis_summary.txt')
    
    try:
        with open(summary_path, 'w') as f:
            f.write("Batch Covariance Analysis Summary\n")
            f.write("=" * 50 + "\n\n")
            
            successful_datasets = [name for name, result in results.items() if result is not None]
            failed_datasets = [name for name, result in results.items() if result is None]
            
            f.write(f"Total datasets: {len(results)}\n")
            f.write(f"Successful: {len(successful_datasets)}\n")
            f.write(f"Failed: {len(failed_datasets)}\n\n")
            
            if failed_datasets:
                f.write("Failed datasets:\n")
                for dataset in failed_datasets:
                    f.write(f"  - {dataset}\n")
                f.write("\n")
            
            # Summary table for successful datasets
            if successful_datasets:
                f.write("Successful Analysis Summary:\n")
                f.write("-" * 40 + "\n")
                f.write(f"{'Dataset':<20} {'IMU':<8} {'Odom':<8} {'Motion Types':<15}\n")
                f.write("-" * 40 + "\n")
                
                for dataset_name in successful_datasets:
                    result = results[dataset_name]
                    imu_count = len(result['imu_results'])
                    odom_count = len(result['odom_results'])
                    motion_types = set(list(result['imu_results'].keys()) + list(result['odom_results'].keys()))
                    motion_str = ', '.join(sorted(motion_types))
                    
                    f.write(f"{dataset_name:<20} {imu_count:<8} {odom_count:<8} {motion_str:<15}\n")
                
                f.write("\n")
                
                # Detailed comparison
                f.write("Detailed Comparison:\n")
                f.write("-" * 40 + "\n")
                
                for dataset_name in successful_datasets:
                    result = results[dataset_name]
                    f.write(f"\nDataset: {dataset_name}\n")
                    
                    # IMU comparison
                    if result['imu_results']:
                        f.write("  IMU Results:\n")
                        for motion_type, imu_result in result['imu_results'].items():
                            yaw_std = (imu_result.yaw_variance ** 0.5) * 180 / 3.14159  # Convert to degrees
                            yaw_rate_std = (imu_result.yaw_rate_variance ** 0.5) * 180 / 3.14159  # Convert to deg/s
                            f.write(f"    {motion_type}: yaw_std={yaw_std:.4f}°, yaw_rate_std={yaw_rate_std:.4f}°/s\n")
                    
                    # Odometry comparison
                    if result['odom_results']:
                        f.write("  Odometry Results:\n")
                        for motion_type, odom_result in result['odom_results'].items():
                            vx_std = odom_result.velocity_x_variance ** 0.5
                            vy_std = odom_result.velocity_y_variance ** 0.5
                            f.write(f"    {motion_type}: vx_std={vx_std:.6f}m/s, vy_std={vy_std:.6f}m/s\n")
        
        print(f"Batch summary saved to {summary_path}")
        
    except Exception as e:
        print(f"Failed to generate batch summary: {e}")


if __name__ == '__main__':
    sys.exit(main())
