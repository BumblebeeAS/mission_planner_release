"""
Common pose utilities for mission_planner_2.
"""

import numpy as np
from bb_perception_msgs.action import ClusterTf
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


def create_clustering_goal(
    in_parent: str,
    in_child: str,
    out_child: str,
    out_parent: str = "world_ned",
    duration: int = 20,
    tf_lookup_interval: float = 0.05,
    cache_size: int = 100,
    min_cluster_size: int = 2,
    min_samples: int = 1,
    use_cache: bool = False,
):
    """Create a ClusterTf goal for collecting and clustering coordinate transforms.

    Args:
        in_parent (str): The parent frame ID for the input transform lookup.
            This is the reference frame from which transforms will be measured.
        in_child (str): The child frame ID for the input transform lookup.
            This is the target frame to which transforms will be measured.
        out_child (str): The child frame ID for the output clustered transform.
            This defines the target frame in the resulting clustered transform.
        out_parent (str, optional): The parent frame ID for the output clustered
            transform. Defaults to "world_ned" (to keep the frame fixed with respect to the world).
        duration (int, optional): The duration in seconds over which to collect
            transforms for clustering. Defaults to 20 seconds.
        tf_lookup_interval (float, optional): The interval in seconds between
            transform lookups during collection. Defaults to 0.05 seconds (20 Hz).
        cache_size (int, optional): The maximum number of transforms to store
            in the cache when use_cache is True. Defaults to 100.
        min_cluster_size (int, optional): The minimum number of transforms
            required to form a cluster during clustering analysis. Defaults to 2.
        min_samples (int, optional): The minimum number of samples required
            for a point to be considered a core point in clustering. Defaults to 1.
        use_cache (bool, optional): Whether to use caching during transform
            collection. If True, uses cache_size; if False, collects as
            many transforms as possible within the duration. Defaults to False.

    Returns:
        ClusterTf.Goal: A configured goal object.

    Example:
        >>> # Create a goal to cluster transforms from base_link to camera
        >>> goal = create_clustering_goal(
        ...     in_parent="base_link",
        ...     in_child="camera_frame",
        ...     out_child="clustered_camera",
        ...     duration=60,
        ...     tf_lookup_interval=0.1,
        ...     min_cluster_size=5
        ... )
        >>>
        >>> # Use the goal with a py_trees_ros action client
        >>> cluster = py_trees_ros.action_clients.FromConstant(
        ...     name="cluster_action",
        ...     action_type=ClusterTf,
        ...     action_name="/auv4/cluster_tf",
        ...     action_goal=goal,
        ... )
        >>>
        >>> # Or create the goal directly in the action client call
        >>> cluster = py_trees_ros.action_clients.FromConstant(
        ...     name="cluster_action",
        ...     action_type=ClusterTf,
        ...     action_name="/auv4/cluster_tf",
        ...     action_goal=create_clustering_goal(
        ...         in_parent="base_link",
        ...         in_child="camera_frame",
        ...         out_child="clustered_camera"
        ...     ),
        ... )
    """
    goal = ClusterTf.Goal()
    goal.input_parent_frame_id = in_parent
    goal.input_child_frame_id = in_child
    goal.output_parent_frame_id = out_parent
    goal.output_child_frame_id = out_child
    goal.clustering_duration = duration
    goal.tf_lookup_interval = tf_lookup_interval
    goal.cache_size = cache_size
    goal.min_cluster_size = min_cluster_size
    goal.min_samples = min_samples
    goal.use_cache = use_cache
    return goal
