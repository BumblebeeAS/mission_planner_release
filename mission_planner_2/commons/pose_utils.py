"""
Common pose utilities for mission_planner_2.
"""

import time
import numpy as np
import rclpy
from rclpy.time import Time as rclpy_time
from geometry_msgs.msg import PoseStamped
import py_trees.console as console
from transforms3d.euler import euler2quat


def create_target_pose(frame):
    """
    Generate a PoseStamped message with all zeros for position and orientation,
    the current time, and the specified frame as the frame_id.

    Args:
        frame (str): The frame ID to use for the PoseStamped message

    Returns:
        PoseStamped: A PoseStamped message with the specified parameters
    """
    pose_stamped = PoseStamped()

    pose_stamped.header.frame_id = frame

    if rclpy.ok():
        node_time = rclpy_time()
        pose_stamped.header.stamp.sec = node_time.seconds_nanoseconds()[0]
        pose_stamped.header.stamp.nanosec = node_time.seconds_nanoseconds()[1]
    else:
        console.logwarn("rclpy has not init, using system time instead of ros time")
        current_time = time.time()
        pose_stamped.header.stamp.sec = int(current_time)
        pose_stamped.header.stamp.nanosec = int(
            (current_time - int(current_time)) * 1e9
        )

    pose_stamped.pose.position.x = 0.0
    pose_stamped.pose.position.y = 0.0
    pose_stamped.pose.position.z = 0.0

    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 1.0

    return pose_stamped


def create_stamped_pose(
    frame_id,
    position_x=0.0,
    position_y=0.0,
    position_z=0.0,
    roll=0.0,
    pitch=0.0,
    yaw=0.0,
):
    """
    Create a PoseStamped message with the given parameters.

    Args:
        frame_id (str): The frame ID for the pose
        position_x (float): X position component
        position_y (float): Y position component
        position_z (float): Z position component
        roll (float): Roll angle in degrees
        pitch (float): Pitch angle in degrees
        yaw (float): Yaw angle in degrees

    Returns:
        PoseStamped: A PoseStamped message with the specified parameters
    """
    pose_stamped = PoseStamped()

    pose_stamped.header.frame_id = frame_id

    if rclpy.ok():
        node_time = rclpy_time()
        pose_stamped.header.stamp.sec = node_time.seconds_nanoseconds()[0]
        pose_stamped.header.stamp.nanosec = node_time.seconds_nanoseconds()[1]
    else:
        console.logwarn("rclpy has not init, using system time instead of ros time")
        current_time = time.time()
        pose_stamped.header.stamp.sec = int(current_time)
        pose_stamped.header.stamp.nanosec = int(
            (current_time - int(current_time)) * 1e9
        )

    pose_stamped.pose.position.x = position_x
    pose_stamped.pose.position.y = position_y
    pose_stamped.pose.position.z = position_z

    # Convert from degrees to radians
    roll_rad = np.radians(roll)
    pitch_rad = np.radians(pitch)
    yaw_rad = np.radians(yaw)

    # Note: transforms3d uses (w, x, y, z) format, but ROS uses (x, y, z, w)
    quat = euler2quat(roll_rad, pitch_rad, yaw_rad, "sxyz")

    # Extract components in ROS order (x, y, z, w)
    pose_stamped.pose.orientation.x = quat[1]
    pose_stamped.pose.orientation.y = quat[2]
    pose_stamped.pose.orientation.z = quat[3]
    pose_stamped.pose.orientation.w = quat[0]

    return pose_stamped
