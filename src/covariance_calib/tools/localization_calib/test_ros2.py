#!/usr/bin/env python3
"""
Simple test script to check ROS2 environment
"""
import subprocess
import sys

def test_ros2():
    print("=== Testing ROS2 Environment ===")
    
    # Test 1: Check if ros2 command exists
    try:
        result = subprocess.run(['which', 'ros2'], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✓ ros2 command found at: {result.stdout.strip()}")
        else:
            print("✗ ros2 command not found in PATH")
            return False
    except Exception as e:
        print(f"✗ Error checking ros2 command: {e}")
        return False
    
    # Test 2: Check ros2 version
    try:
        result = subprocess.run(['ros2', '--version'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print(f"✓ ROS2 version: {result.stdout.strip()}")
        else:
            print(f"✗ Failed to get ros2 version: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print("✗ Timeout getting ros2 version")
        return False
    except Exception as e:
        print(f"✗ Error getting ros2 version: {e}")
        return False
    
    # Test 3: List topics
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            print(f"✓ Found {len(topics)} topics")
            
            # Check for required topics
            required_topics = ['/hardware_layer/imu/data_raw', '/hardware_layer/diff_cont/odom']
            for topic in required_topics:
                if topic in topics:
                    print(f"  ✓ {topic}")
                else:
                    print(f"  ✗ {topic} (missing)")
        else:
            print(f"✗ Failed to list topics: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print("✗ Timeout listing topics")
        return False
    except Exception as e:
        print(f"✗ Error listing topics: {e}")
        return False
    
    print("\n=== Environment Check Complete ===")
    return True

if __name__ == "__main__":
    success = test_ros2()
    if not success:
        print("\nPlease fix the ROS2 environment issues above before running the calibration tool.")
        sys.exit(1)
    else:
        print("\n✓ ROS2 environment looks good! You can now run the calibration tool.")
