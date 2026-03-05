#!/usr/bin/env python3

import os
import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from .utils import quaternion_to_yaw, remove_outliers, validate_bag_data


@dataclass
class SensorData:
    """Container for sensor data with timestamps."""
    timestamps: List[float]
    data: List[any]
    
    def __len__(self):
        return len(self.timestamps)


@dataclass
class MotionSegment:
    """Container for a motion segment with start/end times."""
    start_time: float
    end_time: float
    motion_type: str  # 'stationary', 'spinning', 'straight'
    duration: float
    
    def __post_init__(self):
        self.duration = self.end_time - self.start_time


class DataAnalyzer:
    """Analyzes bag data and detects motion states."""
    
    def __init__(self, config: Dict):
        self.config = config
        self.wheel_velocity_threshold = config.get('stationary_detection', {}).get('wheel_velocity_threshold', 0.01)
        self.min_stationary_duration = config.get('stationary_detection', {}).get('min_stationary_duration', 2.0)
        
        # Data storage
        self.imu_data = {}
        self.odom_data = {}
        self.joint_states_data = {}
        
    def load_bag_data(self, bag_path: str) -> bool:
        """
        Load data from a ROS2 bag file.
        
        Args:
            bag_path: Path to the bag directory
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Find uncompressed database files first
            db_files = []
            compressed_files = []
            
            for file in os.listdir(bag_path):
                if file.endswith('.db3'):
                    db_files.append(os.path.join(bag_path, file))
                elif file.endswith('.db3.zstd'):
                    compressed_files.append(os.path.join(bag_path, file))
            
            # Prioritize existing .db3 files over compressed ones
            if db_files:
                print(f"Found {len(db_files)} uncompressed .db3 files")
            elif compressed_files:
                print(f"Found {len(compressed_files)} compressed files, but no .db3 files")
                print(f"Please decompress them manually first:")
                for compressed_file in compressed_files:
                    print(f"  zstd -d {compressed_file}")
                print(f"Or install zstd: sudo apt install zstd")
                return False
            else:
                print(f"No bag database files found in {bag_path}")
                return False
            
            # Load data using rosbag2_py API
            all_data = self._read_bag_database(bag_path)
            
            # Sort by timestamp
            for topic in all_data:
                all_data[topic].sort(key=lambda x: x[0])  # Sort by timestamp
            
            # Store data
            self.imu_data = SensorData(
                timestamps=[msg[0] for msg in all_data['imu']],
                data=[msg[1] for msg in all_data['imu']]
            )
            self.odom_data = SensorData(
                timestamps=[msg[0] for msg in all_data['odom']],
                data=[msg[1] for msg in all_data['odom']]
            )
            self.joint_states_data = SensorData(
                timestamps=[msg[0] for msg in all_data['joint_states']],
                data=[msg[1] for msg in all_data['joint_states']]
            )
            
            print(f"Loaded bag data from {bag_path}:")
            print(f"  IMU messages: {len(self.imu_data)}")
            print(f"  Odometry messages: {len(self.odom_data)}")
            print(f"  Joint states messages: {len(self.joint_states_data)}")
            
            return True
            
        except Exception as e:
            print(f"Error loading bag data: {e}")
            return False
    
    def _read_bag_database(self, bag_path: str) -> Dict[str, List]:
        """
        Read messages from a bag directory using rosbag2_py.
        Only reads .db3 files, ignores compressed files.
        
        Args:
            bag_path: Path to the bag directory
            
        Returns:
            Dictionary with topic data
        """
        data = {'imu': [], 'odom': [], 'joint_states': []}
        
        try:
            # Check if .db3 files exist
            db3_files = []
            for file in os.listdir(bag_path):
                if file.endswith('.db3'):
                    db3_files.append(file)
            
            if not db3_files:
                print(f"No .db3 files found in {bag_path}")
                print(f"Please ensure bag files are decompressed to .db3 format")
                return data
            
            print(f"Found {len(db3_files)} .db3 files: {db3_files}")
            
            # Setup storage options for bag directory
            storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
            
            # Create reader
            reader = SequentialReader()
            reader.open(storage_options, converter_options)
            
            # Get topic types
            topic_types = reader.get_all_topics_and_types()
            type_map = {topic.name: topic.type for topic in topic_types}
            
            # Read messages
            while reader.has_next():
                (topic, data_bytes, timestamp) = reader.read_next()
                
                # Convert timestamp from nanoseconds to seconds
                timestamp_sec = timestamp / 1e9
                
                try:
                    # Get message type and deserialize
                    if topic in type_map:
                        msg_type = type_map[topic]
                        msg_class = get_message(msg_type)
                        msg = deserialize_message(data_bytes, msg_class)
                        
                        # Store based on topic
                        if topic == self.config.get('imu_topic', '/hardware_layer/imu/data_raw'):
                            data['imu'].append((timestamp_sec, msg))
                        elif topic == self.config.get('odom_topic', '/hardware_layer/diff_cont/odom'):
                            data['odom'].append((timestamp_sec, msg))
                        elif topic == self.config.get('joint_states_topic', '/hardware_layer/joint_states'):
                            data['joint_states'].append((timestamp_sec, msg))
                            
                except Exception as e:
                    print(f"Error deserializing message from topic {topic}: {e}")
                    continue
            
            # Note: SequentialReader doesn't have a close() method
            
        except Exception as e:
            print(f"Error reading bag {bag_path}: {e}")
        
        return data
    
    def detect_stationary_periods(self) -> List[MotionSegment]:
        """
        Detect stationary periods based on joint states (wheel velocities).
        
        Returns:
            List of stationary motion segments
        """
        if len(self.joint_states_data) == 0:
            print("No joint states data available for stationary detection")
            return []
        
        stationary_segments = []
        current_stationary_start = None
        
        for i, (timestamp, joint_msg) in enumerate(zip(self.joint_states_data.timestamps, 
                                                      self.joint_states_data.data)):
            # Calculate wheel velocities (assuming differential drive)
            wheel_velocities = joint_msg.velocity
            
            if len(wheel_velocities) < 2:
                continue
            
            # Check if robot is stationary (both wheels below threshold)
            is_stationary = all(abs(vel) < self.wheel_velocity_threshold for vel in wheel_velocities)
            
            if is_stationary:
                if current_stationary_start is None:
                    current_stationary_start = timestamp
            else:
                if current_stationary_start is not None:
                    # End of stationary period
                    duration = timestamp - current_stationary_start
                    if duration >= self.min_stationary_duration:
                        segment = MotionSegment(
                            start_time=current_stationary_start,
                            end_time=timestamp,
                            motion_type='stationary',
                            duration=duration
                        )
                        stationary_segments.append(segment)
                    current_stationary_start = None
        
        # Handle case where data ends while stationary
        if current_stationary_start is not None:
            end_time = self.joint_states_data.timestamps[-1]
            duration = end_time - current_stationary_start
            if duration >= self.min_stationary_duration:
                segment = MotionSegment(
                    start_time=current_stationary_start,
                    end_time=end_time,
                    motion_type='stationary',
                    duration=duration
                )
                stationary_segments.append(segment)
        
        print(f"Detected {len(stationary_segments)} stationary periods:")
        for i, segment in enumerate(stationary_segments):
            print(f"  Period {i+1}: {segment.duration:.2f}s ({segment.start_time:.2f} - {segment.end_time:.2f})")
        
        return stationary_segments
    
    def extract_imu_data_for_segment(self, segment: MotionSegment) -> Tuple[np.ndarray, np.ndarray]:
        """
        Extract yaw and yaw_rate data for a specific time segment.
        
        Args:
            segment: Motion segment to extract data for
            
        Returns:
            Tuple of (yaw_data, yaw_rate_data) as numpy arrays
        """
        yaw_data = []
        yaw_rate_data = []
        
        for timestamp, imu_msg in zip(self.imu_data.timestamps, self.imu_data.data):
            if segment.start_time <= timestamp <= segment.end_time:
                # Extract yaw from quaternion
                yaw = quaternion_to_yaw(imu_msg.orientation)
                yaw_data.append(yaw)
                
                # Extract yaw rate (angular velocity z)
                yaw_rate = imu_msg.angular_velocity.z
                yaw_rate_data.append(yaw_rate)
        
        return np.array(yaw_data), np.array(yaw_rate_data)
    
    def extract_odom_data_for_segment(self, segment: MotionSegment) -> Tuple[np.ndarray, np.ndarray]:
        """
        Extract velocity_x and velocity_y data for a specific time segment.
        
        Args:
            segment: Motion segment to extract data for
            
        Returns:
            Tuple of (velocity_x_data, velocity_y_data) as numpy arrays
        """
        vx_data = []
        vy_data = []
        
        for timestamp, odom_msg in zip(self.odom_data.timestamps, self.odom_data.data):
            if segment.start_time <= timestamp <= segment.end_time:
                vx_data.append(odom_msg.twist.twist.linear.x)
                vy_data.append(odom_msg.twist.twist.linear.y)
        
        return np.array(vx_data), np.array(vy_data)
    
    def filter_outliers_from_data(self, data1: np.ndarray, data2: np.ndarray, 
                                 threshold: float = 3.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        Remove outliers from paired datasets.
        
        Args:
            data1: First dataset
            data2: Second dataset  
            threshold: Standard deviation threshold for outlier detection
            
        Returns:
            Tuple of filtered datasets
        """
        if len(data1) != len(data2):
            raise ValueError("Datasets must have the same length")
        
        # Remove outliers from data1
        _, mask1 = remove_outliers(data1, threshold)
        
        # Remove outliers from data2
        _, mask2 = remove_outliers(data2, threshold)
        
        # Combine masks (keep only points that are not outliers in either dataset)
        combined_mask = mask1 & mask2
        
        filtered_data1 = data1[combined_mask]
        filtered_data2 = data2[combined_mask]
        
        removed_count = len(data1) - len(filtered_data1)
        if removed_count > 0:
            print(f"Removed {removed_count} outliers ({removed_count/len(data1)*100:.1f}%)")
        
        return filtered_data1, filtered_data2
    
    def get_data_summary(self) -> Dict:
        """
        Get summary statistics of loaded data.
        
        Returns:
            Dictionary with data summary
        """
        summary = {
            'imu': {
                'count': len(self.imu_data),
                'duration': 0,
                'frequency': 0
            },
            'odom': {
                'count': len(self.odom_data),
                'duration': 0,
                'frequency': 0
            },
            'joint_states': {
                'count': len(self.joint_states_data),
                'duration': 0,
                'frequency': 0
            }
        }
        
        # Calculate durations and frequencies
        for topic, data in [('imu', self.imu_data), ('odom', self.odom_data), 
                           ('joint_states', self.joint_states_data)]:
            if len(data) > 1:
                duration = data.timestamps[-1] - data.timestamps[0]
                frequency = len(data) / duration if duration > 0 else 0
                summary[topic]['duration'] = duration
                summary[topic]['frequency'] = frequency
        
        return summary
