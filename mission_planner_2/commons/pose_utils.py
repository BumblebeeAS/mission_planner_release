"""
Common pose utilities for mission_planner_2.
"""

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from transforms3d.euler import euler2quat


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

    # Transform lookups will yield latest transform
    pose_stamped.header.stamp = Time()

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
