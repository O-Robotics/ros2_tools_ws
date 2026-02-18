#!/usr/bin/env python3
"""
Covariance calculation script for localization sensor calibration.
Analyzes recorded bag data and calculates optimal covariance values.
"""

import argparse
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# Import the existing calculation logic
sys.path.append(str(Path(__file__).parent.parent / 'tools' / 'localization_calib' / 'calculate'))
from calculate_cov import CovarianceCalculator, BagReader, generate_summary_report


def main():
    parser = argparse.ArgumentParser(
        description="Calculate covariance values from calibration bag data",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Standard analysis
  ros2 run covariance_calib calculate_cov --bags_root ~/calib_data/2026-02-18/
  
  # Custom segmentation
  ros2 run covariance_calib calculate_cov --bags_root ~/calib_data/2026-02-18/ --segment_mode raw
  
  # Specific phases only
  ros2 run covariance_calib calculate_cov --bags_root ~/calib_data/2026-02-18/ --run stationary slow_spin
        """
    )
    
    parser.add_argument('--bags_root', type=str, required=True,
                       help='Root directory containing calibration bags')
    parser.add_argument('--run', nargs='+', 
                       choices=['stationary', 'slow_spin', 'slow_straight'],
                       help='Specific phases to analyze (default: all available)')
    parser.add_argument('--imu_topic', type=str, 
                       default='/hardware_layer/imu/data_raw',
                       help='IMU topic name')
    parser.add_argument('--odom_topic', type=str,
                       default='/hardware_layer/diff_cont/odom', 
                       help='Odometry topic name')
    parser.add_argument('--out_dir', type=str,
                       help='Output directory (default: auto-generated)')
    parser.add_argument('--segment_mode', type=str, default='auto',
                       choices=['auto', 'trim_only', 'threshold_only', 'raw'],
                       help='Data segmentation strategy')
    parser.add_argument('--min_samples', type=int,
                       help='Minimum samples required (overrides config)')
    parser.add_argument('--wz_min', type=float,
                       help='Minimum angular velocity for slow_spin (rad/s)')
    parser.add_argument('--vx_min', type=float,
                       help='Minimum linear velocity for slow_straight (m/s)')
    
    args = parser.parse_args()
    
    try:
        # Get config from ROS2 package share directory
        try:
            package_share_directory = get_package_share_directory('covariance_calib')
            config_path = Path(package_share_directory) / 'config' / 'config.yaml'
        except Exception:
            # Fallback to local path for development
            config_path = Path(__file__).parent.parent / 'tools' / 'localization_calib' / 'calculate' / 'config.yaml'
        
        # Initialize calculator
        calculator = CovarianceCalculator(str(config_path))
        
        # Set up paths
        bags_root = Path(args.bags_root)
        if not bags_root.exists():
            print(f"Error: Bags directory not found: {bags_root}")
            sys.exit(1)
        
        # Auto-generate output directory if not specified
        if args.out_dir:
            output_dir = Path(args.out_dir)
        else:
            output_dir = bags_root.parent / 'results' / bags_root.name
        
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Create subdirectories
        imu_dir = output_dir / 'imu'
        odom_dir = output_dir / 'odom'
        imu_dir.mkdir(exist_ok=True)
        odom_dir.mkdir(exist_ok=True)
        
        print(f"Input: {bags_root}")
        print(f"Output: {output_dir}")
        print(f"Segmentation mode: {args.segment_mode}")
        
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
                    output_file = imu_dir / f'{phase}.json'
                    with open(output_file, 'w') as f:
                        import json
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
                    output_file = odom_dir / f'{phase}.json'
                    with open(output_file, 'w') as f:
                        import json
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
            print(f"✓ View the report: {output_dir}/report.md")
        else:
            print("✗ No valid results generated")
            sys.exit(1)
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
