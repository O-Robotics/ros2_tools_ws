#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Twist, Vector3
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class SimpleOdomPublisher(Node):
    """
    Simple odometry publisher based on IMU integration
    This is a basic implementation for testing purposes
    """
    
    def __init__(self):
        super().__init__('simple_odom_publisher')
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        
        # State variables - start with small non-zero values for EKF
        self.x = 0.001  # Small initial position to help EKF
        self.y = 0.001
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        
        self.last_time = self.get_clock().now()
        
        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_odom)  # 10 Hz
        
        self.get_logger().info('Simple odometry publisher started')
    
    def imu_callback(self, msg):
        """
        Process IMU data to estimate motion
        This is a very basic implementation
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 0:
            # Use angular velocity from IMU
            self.vth = msg.angular_velocity.z
            
            # Simple integration (this would be much better with wheel encoders)
            # For static testing, add tiny random motion to help EKF
            import random
            self.vx = random.uniform(-0.001, 0.001)  # Tiny random motion for EKF
            self.vy = random.uniform(-0.001, 0.001)
            
            # Update pose
            delta_theta = self.vth * dt
            delta_x = self.vx * math.cos(self.theta) * dt
            delta_y = self.vx * math.sin(self.theta) * dt
            
            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta
        
        self.last_time = current_time
    
    def publish_odom(self):
        """
        Publish odometry message and TF
        """
        current_time = self.get_clock().now()
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation = self.yaw_to_quaternion(self.theta)
        
        # Velocity
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        
        # Covariance (simple diagonal)
        odom.pose.covariance[0] = 0.1   # x
        odom.pose.covariance[7] = 0.1   # y
        odom.pose.covariance[35] = 0.1  # yaw
        odom.twist.covariance[0] = 0.1  # vx
        odom.twist.covariance[35] = 0.1 # vyaw
        
        # Publish odometry
        self.odom_pub.publish(odom)
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.yaw_to_quaternion(self.theta)
        
        self.tf_broadcaster.sendTransform(t)
    
    def yaw_to_quaternion(self, yaw):
        """
        Convert yaw angle to quaternion
        """
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdomPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
