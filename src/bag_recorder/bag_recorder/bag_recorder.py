#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import os
from datetime import datetime
import signal
import sys
import atexit


class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')
        
        # Declare parameters for current sensors
        self.declare_parameter('output_dir', '~/ros2_bags')
        self.declare_parameter('bag_name', '')
        
        # Current sensor topics - easily extensible
        self.declare_parameter('topics', ['/imu/data_raw', '/fix'])
        
        # Future sensor topics (commented for extension):
        # Camera topics: ['/camera/image_raw', '/camera/camera_info']
        # Wheel odometry: ['/wheel_odom', '/wheel_speeds']
        # Lidar: ['/scan', '/pointcloud']
        # Additional IMU: ['/imu/filtered', '/imu/mag']
        # GPS RTK: ['/rtk/fix', '/rtk/heading']
        
        self.declare_parameter('max_bag_duration', 0)  # 0 means no limit
        self.declare_parameter('max_bag_size', 0)  # 0 means no limit (in MB)
        self.declare_parameter('compression_mode', 'none')  # none, file, message
        self.declare_parameter('compression_format', 'zstd')  # zstd, lz4
        self.declare_parameter('storage_format', 'sqlite3')  # sqlite3, mcap
        
        # Get parameters
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.bag_name = self.get_parameter('bag_name').get_parameter_value().string_value
        self.topics = self.get_parameter('topics').get_parameter_value().string_array_value
        self.max_duration = self.get_parameter('max_bag_duration').get_parameter_value().integer_value
        self.max_size = self.get_parameter('max_bag_size').get_parameter_value().integer_value
        self.compression_mode = self.get_parameter('compression_mode').get_parameter_value().string_value
        self.compression_format = self.get_parameter('compression_format').get_parameter_value().string_value
        self.storage_format = self.get_parameter('storage_format').get_parameter_value().string_value
        
        # Expand home directory
        self.output_dir = os.path.expanduser(self.output_dir)
        
        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Generate bag name if not provided
        if not self.bag_name:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.bag_name = f"sensor_data_{timestamp}"  # Generic name for all sensors
        
        self.bag_path = os.path.join(self.output_dir, self.bag_name)
        
        # Build ros2 bag record command
        self.cmd = ['ros2', 'bag', 'record', '-o', self.bag_path]
        
        # Add duration limit if specified
        if self.max_duration > 0:
            self.cmd.extend(['--max-bag-duration', str(self.max_duration)])
        
        # Add size limit if specified
        if self.max_size > 0:
            self.cmd.extend(['--max-bag-size', str(self.max_size * 1024 * 1024)])  # Convert MB to bytes
        
        # Add compression settings
        if self.compression_mode != 'none':
            self.cmd.extend(['--compression-mode', self.compression_mode])
            self.cmd.extend(['--compression-format', self.compression_format])
        
        # Add storage format
        self.cmd.extend(['--storage', self.storage_format])
        
        # Add topics
        self.cmd.extend(self.topics)
        
        self.get_logger().info(f"Starting sensor data bag recording with command: {' '.join(self.cmd)}")
        self.get_logger().info(f"Recording topics: {', '.join(self.topics)}")
        self.get_logger().info(f"Output path: {self.bag_path}")
        if self.max_duration > 0:
            self.get_logger().info(f"Max duration: {self.max_duration} seconds")
        if self.max_size > 0:
            self.get_logger().info(f"Max size: {self.max_size} MB")
        if self.compression_mode != 'none':
            self.get_logger().info(f"Compression: {self.compression_mode} ({self.compression_format})")
        self.get_logger().info(f"Storage format: {self.storage_format}")
        
        # Start recording
        self.process = None
        self.start_recording()
        
        # Register atexit handler for cleanup (backup shutdown mechanism)
        atexit.register(self.stop_recording)
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def start_recording(self):
        """Start the bag recording process"""
        try:
            self.process = subprocess.Popen(self.cmd)
            self.get_logger().info("Sensor data bag recording started successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to start bag recording: {str(e)}")
            sys.exit(1)
    
    def stop_recording(self):
        """Stop the bag recording process"""
        if self.process and self.process.poll() is None:
            self.get_logger().info("Stopping sensor data bag recording...")
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Recording process didn't terminate gracefully, forcing kill")
                self.process.kill()
            self.get_logger().info("Sensor data bag recording stopped")
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.get_logger().info(f"Received signal {signum}, shutting down...")
        self.stop_recording()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    recorder = None
    
    try:
        recorder = BagRecorder()
        
        # Keep the node alive
        while rclpy.ok():
            try:
                rclpy.spin_once(recorder, timeout_sec=1.0)
                
                # Check if the recording process is still running
                if recorder.process and recorder.process.poll() is not None:
                    recorder.get_logger().info("Recording process finished")
                    break
                    
            except KeyboardInterrupt:
                recorder.get_logger().info("Received keyboard interrupt, shutting down...")
                break
    
    except Exception as e:
        if recorder:
            recorder.get_logger().error(f"Error in bag recorder: {e}")
        else:
            # Fallback if recorder wasn't created yet
            print(f"Error initializing bag recorder: {e}")
    
    finally:
        if recorder:
            recorder.stop_recording()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
