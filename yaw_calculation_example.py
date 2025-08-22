#!/usr/bin/env python3
"""
Yaw angle calculation example for frontend drawn paths
Calculate robot heading angle from GPS coordinates (latitude, longitude)
"""

import math
import json

def calculate_yaw_from_gps_points(lat1, lon1, lat2, lon2):
    """
    Calculate yaw angle from two GPS points (direction from point1 to point2)
    
    Args:
        lat1, lon1: Starting point latitude and longitude (degrees)
        lat2, lon2: Target point latitude and longitude (degrees)
    
    Returns:
        yaw: Yaw angle in radians (0=East, π/2=North, π=West, 3π/2=South)
    """
    # Convert to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    # Calculate longitude difference
    dlon = lon2_rad - lon1_rad
    
    # Calculate bearing angle
    y = math.sin(dlon) * math.cos(lat2_rad)
    x = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
         math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon))
    
    # atan2 returns angle range [-π, π]
    bearing = math.atan2(y, x)
    
    # Convert to [0, 2π] range
    if bearing < 0:
        bearing += 2 * math.pi
    
    return bearing

def calculate_yaw_degrees(lat1, lon1, lat2, lon2):
    """
    Calculate yaw angle in degrees
    """
    yaw_rad = calculate_yaw_from_gps_points(lat1, lon1, lat2, lon2)
    return math.degrees(yaw_rad)

def process_waypoint_path(waypoints):
    """
    Process entire path, calculate yaw angle for each waypoint
    
    Args:
        waypoints: [{"latitude": lat, "longitude": lon, "altitude": alt}, ...]
    
    Returns:
        waypoints_with_yaw: Waypoint list with added yaw field
    """
    if len(waypoints) < 2:
        return waypoints
    
    result = []
    
    for i in range(len(waypoints)):
        waypoint = waypoints[i].copy()
        
        if i == 0:
            # First point: use direction pointing to next point
            yaw = calculate_yaw_from_gps_points(
                waypoints[i]["latitude"], waypoints[i]["longitude"],
                waypoints[i+1]["latitude"], waypoints[i+1]["longitude"]
            )
        elif i == len(waypoints) - 1:
            # Last point: use direction from previous point
            yaw = calculate_yaw_from_gps_points(
                waypoints[i-1]["latitude"], waypoints[i-1]["longitude"],
                waypoints[i]["latitude"], waypoints[i]["longitude"]
            )
        else:
            # Middle point: use average direction from previous to next point
            yaw1 = calculate_yaw_from_gps_points(
                waypoints[i-1]["latitude"], waypoints[i-1]["longitude"],
                waypoints[i]["latitude"], waypoints[i]["longitude"]
            )
            yaw2 = calculate_yaw_from_gps_points(
                waypoints[i]["latitude"], waypoints[i]["longitude"],
                waypoints[i+1]["latitude"], waypoints[i+1]["longitude"]
            )
            # Angle averaging (handle angle wraparound)
            yaw = angle_average(yaw1, yaw2)
        
        waypoint["yaw"] = yaw
        result.append(waypoint)
    
    return result

def angle_average(angle1, angle2):
    """
    Calculate average of two angles (handle 0/2π boundary crossing)
    """
    # Convert to unit vectors
    x1, y1 = math.cos(angle1), math.sin(angle1)
    x2, y2 = math.cos(angle2), math.sin(angle2)
    
    # Average vector
    avg_x = (x1 + x2) / 2
    avg_y = (y1 + y2) / 2
    
    # Convert back to angle
    avg_angle = math.atan2(avg_y, avg_x)
    if avg_angle < 0:
        avg_angle += 2 * math.pi
    
    return avg_angle

def direction_to_string(yaw_rad):
    """
    Convert yaw angle to direction description
    """
    yaw_deg = math.degrees(yaw_rad)
    
    if yaw_deg < 22.5 or yaw_deg >= 337.5:
        return "East"
    elif yaw_deg < 67.5:
        return "Northeast"
    elif yaw_deg < 112.5:
        return "North"
    elif yaw_deg < 157.5:
        return "Northwest"
    elif yaw_deg < 202.5:
        return "West"
    elif yaw_deg < 247.5:
        return "Southwest"
    elif yaw_deg < 292.5:
        return "South"
    else:
        return "Southeast"

# Example usage
if __name__ == "__main__":
    # Example: waypoints drawn by frontend
    frontend_waypoints = [
        {"latitude": 56.164379, "longitude": 10.145631, "altitude": 114.3},
        {"latitude": 56.164400, "longitude": 10.145650, "altitude": 114.5},
        {"latitude": 56.164420, "longitude": 10.145680, "altitude": 114.2},
        {"latitude": 56.164450, "longitude": 10.145700, "altitude": 114.0},
    ]
    
    # Calculate yaw angle for each point
    waypoints_with_yaw = process_waypoint_path(frontend_waypoints)
    
    # Output results
    print("Processed waypoints (with yaw angles):")
    print("=" * 60)
    for i, wp in enumerate(waypoints_with_yaw):
        yaw_deg = math.degrees(wp["yaw"])
        direction = direction_to_string(wp["yaw"])
        print(f"Waypoint {i+1}:")
        print(f"  Position: ({wp['latitude']:.6f}, {wp['longitude']:.6f})")
        print(f"  Altitude: {wp['altitude']:.1f}m")
        print(f"  Yaw: {wp['yaw']:.3f} radians ({yaw_deg:.1f}°)")
        print(f"  Direction: {direction}")
        print()
    
    # Generate ROS2 navigation YAML format
    print("ROS2 Navigation YAML format:")
    print("=" * 30)
    print("waypoints:")
    for wp in waypoints_with_yaw:
        print(f"- latitude: {wp['latitude']}")
        print(f"  longitude: {wp['longitude']}")
        print(f"  yaw: {wp['yaw']:.3f}")
