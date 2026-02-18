#!/usr/bin/env python3
"""
Calibration data recording script for localization sensors.
Records ROS2 bag data in different motion phases for covariance calibration.
"""

import argparse
import os
import sys
import yaml
import subprocess
import time
from datetime import datetime
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


class CalibrationRecorder:
    def __init__(self, config_file="topics.yaml"):
        """Initialize the calibration recorder with topic configuration."""
        try:
            # Get config from ROS2 package share directory
            package_share_directory = get_package_share_directory('covariance_calib')
            self.config_path = Path(package_share_directory) / 'config' / config_file
        except Exception:
            # Fallback to local path for development
            self.config_path = Path(__file__).parent.parent / 'tools' / 'localization_calib' / 'record' / config_file
        
        if not self.config_path.exists():
            raise FileNotFoundError(f"Configuration file not found: {self.config_path}")
            
        with open(self.config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        self.phases = self.config['phases']
        self.defaults = self.config['defaults']
        
    def get_topics_for_phase(self, phase):
        """Get the list of topics to record for a specific phase."""
        if phase not in self.phases:
            raise ValueError(f"Unknown phase: {phase}. Available phases: {list(self.phases.keys())}")
            
        # Start with required topics
        topics = self.defaults['required'].copy()
        
        # Add optional topics (you might want to make this configurable)
        topics.extend(self.defaults['optional'])
        
        # Add phase-specific topics
        phase_config = self.phases[phase]
        topics.extend(phase_config.get('add', []))
        
        # Remove phase-specific topics
        for topic in phase_config.get('remove', []):
            if topic in topics:
                topics.remove(topic)
                
        return topics
        
    def create_output_dir(self, base_dir, date_str=None):
        """Create output directory structure."""
        if date_str is None:
            date_str = datetime.now().strftime("%Y-%m-%d")
            
        output_dir = Path(base_dir) / date_str
        output_dir.mkdir(parents=True, exist_ok=True)
        return output_dir
        
    def record_phase(self, phase, duration, output_dir, bag_name=None):
        """Record a single calibration phase."""
        if bag_name is None:
            bag_name = f"calib_{phase}"
            
        bag_path = output_dir / bag_name
        topics = self.get_topics_for_phase(phase)
        
        # Build ros2 bag record command
        cmd = [
            'ros2', 'bag', 'record',
            '-o', str(bag_path),
            '--max-bag-duration', str(duration)
        ]
        
        # Add compression settings if specified
        recording_config = self.config.get('recording', {})
        if recording_config.get('compression_mode', 'none') != 'none':
            cmd.extend(['--compression-mode', recording_config['compression_mode']])
            cmd.extend(['--compression-format', recording_config['compression_format']])
            
        # Add storage format
        cmd.extend(['--storage', recording_config.get('storage_format', 'sqlite3')])
        
        # Add topics
        cmd.extend(topics)
        
        print(f"\n=== Recording Phase: {phase} ===")
        print(f"Duration: {duration} seconds")
        print(f"Output: {bag_path}")
        print(f"Topics ({len(topics)}): {', '.join(topics)}")
        print(f"Command: {' '.join(cmd)}")
        
        # Show phase description and recommended actions
        phase_config = self.phases[phase]
        if 'description' in phase_config:
            print(f"\nDescription: {phase_config['description']}")
            
        # Give user time to prepare
        print(f"\nStarting recording in 5 seconds...")
        print("Make sure the robot is in the correct state for this phase!")
        
        if phase == 'stationary':
            print("-> Robot should be completely stationary")
        elif phase == 'slow_spin':
            print("-> Robot should be rotating slowly (< 0.5 rad/s)")
        elif phase == 'slow_straight':
            print("-> Robot should be moving straight slowly (< 0.3 m/s)")
            
        for i in range(5, 0, -1):
            print(f"{i}...")
            time.sleep(1)
            
        print("Recording started!")
        
        try:
            # Start recording
            process = subprocess.Popen(cmd)
            
            # Wait for completion
            process.wait()
            
            if process.returncode == 0:
                print(f"✓ Successfully recorded {phase} phase to {bag_path}")
                return True
            else:
                print(f"✗ Recording failed with return code {process.returncode}")
                return False
                
        except KeyboardInterrupt:
            print("\nRecording interrupted by user")
            if process:
                process.terminate()
                process.wait()
            return False
        except Exception as e:
            print(f"✗ Recording failed: {e}")
            return False
            
    def record_all_phases(self, base_duration, output_dir):
        """Record all calibration phases."""
        phases_to_record = ['stationary', 'slow_spin', 'slow_straight']
        results = {}
        
        for phase in phases_to_record:
            # Use phase-specific duration if available
            phase_config = self.phases[phase]
            duration = phase_config.get('duration_recommended', base_duration)
            
            print(f"\n{'='*60}")
            print(f"Phase {phases_to_record.index(phase) + 1}/{len(phases_to_record)}: {phase.upper()}")
            print(f"{'='*60}")
            
            success = self.record_phase(phase, duration, output_dir)
            results[phase] = success
            
            if not success:
                print(f"Failed to record {phase}. Continue with next phase? (y/n): ", end='')
                if input().lower() != 'y':
                    break
                    
        return results
        
    def check_prerequisites(self):
        """Check if ROS2 and required topics are available."""
        try:
            # Check if ros2 command is available (use 'ros2 topic list' instead of --version)
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode != 0:
                print("✗ ROS2 not found or not working. Make sure ROS2 is installed and sourced.")
                return False
                
        except (subprocess.TimeoutExpired, FileNotFoundError):
            print("✗ ROS2 not found. Make sure ROS2 is installed and sourced.")
            return False
            
        # Check if required topics are available
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode != 0:
                print("✗ Failed to list ROS2 topics")
                return False
                
            available_topics = set(result.stdout.strip().split('\n'))
            required_topics = set(self.defaults['required'])
            
            missing_topics = required_topics - available_topics
            if missing_topics:
                print(f"✗ Missing required topics: {', '.join(missing_topics)}")
                print("Make sure your localization system is running!")
                return False
                
        except subprocess.TimeoutExpired:
            print("✗ Timeout while checking topics")
            return False
            
        print("✓ Prerequisites check passed")
        return True


def main():
    parser = argparse.ArgumentParser(
        description="Record calibration data for localization sensors",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Record all phases with default durations
  ros2 run covariance_calib record_calib --all
  
  # Record specific phase
  ros2 run covariance_calib record_calib --phase stationary --duration 60
  
  # Custom output directory
  ros2 run covariance_calib record_calib --all --output ~/calib_data/
        """
    )
    
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--phase', choices=['stationary', 'slow_spin', 'slow_straight'],
                      help='Record specific calibration phase')
    group.add_argument('--all', action='store_true',
                      help='Record all calibration phases sequentially')
    
    parser.add_argument('--duration', type=int, default=120,
                       help='Recording duration in seconds (default: 120)')
    parser.add_argument('--output', type=str, default='data/bags',
                       help='Output directory for bag files (default: data/bags)')
    
    args = parser.parse_args()
    
    try:
        recorder = CalibrationRecorder()
        
        # Check prerequisites
        if not recorder.check_prerequisites():
            sys.exit(1)
        
        # Create output directory
        output_dir = recorder.create_output_dir(args.output)
        print(f"Output directory: {output_dir}")
        
        if args.all:
            print("Recording all calibration phases...")
            results = recorder.record_all_phases(args.duration, output_dir)
            
            # Summary
            print(f"\n{'='*60}")
            print("RECORDING SUMMARY")
            print(f"{'='*60}")
            for phase, success in results.items():
                status = "✓ SUCCESS" if success else "✗ FAILED"
                print(f"{phase:15} : {status}")
                
        else:
            print(f"Recording {args.phase} phase...")
            success = recorder.record_phase(args.phase, args.duration, output_dir)
            if success:
                print(f"\n✓ Recording completed successfully")
            else:
                print(f"\n✗ Recording failed")
                sys.exit(1)
                
    except KeyboardInterrupt:
        print("\nRecording interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
