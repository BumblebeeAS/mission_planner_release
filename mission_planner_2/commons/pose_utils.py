"""
Common pose utilities for mission_planner_2.
"""

import numpy as np
from bb_perception_msgs.action import ClusterTf
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from transforms3d.euler import euler2quat
from bb_controls_msgs.srv import Limits


def create_stamped_pose(
    frame_id,
    position_x=0.0,
    position_y=0.0,
    position_z=0.0,
    roll=0.0,
    pitch=0.0,
    yaw=0.0,
    use_radians=False,
):
    """
    Create a PoseStamped message with the given parameters.

    Args:
        frame_id (str): The frame ID for the pose
        position_x (float): X position component
        position_y (float): Y position component
        position_z (float): Z position component
        roll (float): Roll angle
        pitch (float): Pitch angle
        yaw (float): Yaw angle
        use_radians (bool): If True, roll, pitch, and yaw are interpreted as radians.

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
    if not use_radians:
        roll = np.radians(roll)
        pitch = np.radians(pitch)
        yaw = np.radians(yaw)

    quat = euler2quat(roll, pitch, yaw, "sxyz")

    # Extract components in ROS order (x, y, z, w)
    pose_stamped.pose.orientation.x = quat[1]
    pose_stamped.pose.orientation.y = quat[2]
    pose_stamped.pose.orientation.z = quat[3]
    pose_stamped.pose.orientation.w = quat[0]

    return pose_stamped


def create_clustering_goal(
    in_children: str | list[str],
    out_children: str | list[str],
    out_parents: str | list[str] = "world_ned",
    duration: int = 20,
    tf_lookup_interval: float = 0.05,
    cache_size: int = 100,
    min_cluster_size: int = 2,
    min_samples: int = 1,
    use_cache: bool = False,
    persistent: bool = False,
):
    """Create a ClusterTf goal for collecting and clustering coordinate transforms.

    Args:
       in_children (str | list[str]): The child frame ID(s) for the input transform lookup.
           These are the target frames to which transforms will be measured.
           Can be a single frame ID string or a list of frame IDs.
       out_children (str | list[str]): The child frame ID(s) for the output clustered transform(s).
           These define the target frames in the resulting clustered transforms.
           Can be a single frame ID string or a list of frame IDs.
       out_parents (str | list[str], optional): The parent frame ID(s) for the output clustered
           transform(s). If a single string is provided, it will be used for all output transforms.
           If a list is provided, it should match the length of out_children.
           Defaults to "world_ned" (to keep the frames fixed with respect to the world).
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
       persistent (bool, optional): Whether the cache is persisted between distinct
           action calls. If True, then cache is saved and reused for subsequent calls.
           Defaults to False.

    Returns:
       ClusterTf.Goal: A configured goal object.

    Example:
       >>> # Create a goal to cluster transforms for a single frame
       >>> goal = create_clustering_goal(
       ...     in_children="camera_frame",
       ...     out_children="clustered_camera",
       ...     duration=60,
       ...     tf_lookup_interval=0.1,
       ...     min_cluster_size=5
       ... )
       >>>
       >>> # Create a goal to cluster transforms for multiple frames
       >>> goal = create_clustering_goal(
       ...     in_children=["camera_frame", "lidar_frame"],
       ...     out_children=["clustered_camera", "clustered_lidar"],
       ...     out_parents=["base_link", "base_link"],
       ...     duration=30
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
       ...         in_children="camera_frame",
       ...         out_children="clustered_camera"
       ...     ),
       ... )
    """
    if isinstance(in_children, str):
        in_children = [in_children]

    if isinstance(out_children, str):
        out_children = [out_children]

    if isinstance(out_parents, str):
        out_parents = [out_parents] * len(out_children)

    # To be used with modified ClusterTf goal that can support multiple children
    goal = ClusterTf.Goal()
    goal.input_child_frame_ids = in_children
    goal.output_child_frame_ids = out_children
    goal.output_parent_frame_ids = out_parents
    goal.clustering_duration = duration
    goal.tf_lookup_interval = tf_lookup_interval
    goal.cache_size = cache_size
    goal.min_cluster_size = min_cluster_size
    goal.min_samples = min_samples
    goal.use_cache = use_cache
    goal.persistent = persistent
    return goal


def create_slalom_clustering_goal(duration=20, min_cluster_size=10, min_samples=10):
    """Create a ClusterTf goal for slalom clustering.

    Args:
       duration (int, optional): The duration in seconds over which to collect
           transforms for clustering. Defaults to 20 seconds.
       min_cluster_size (int, optional): The minimum number of transforms
           required to form a cluster during clustering analysis. Defaults to 10.
       min_samples (int, optional): The minimum number of samples required
           for a point to be considered a core point in clustering. Defaults to 10.

    Returns:
       ClusterTf.Goal: A configured goal object.

    Note:
       The frame IDs are set to dummy values since the action server loads the
       actual frame configuration from ROS parameters. Only the clustering
       behavior needs to be configured through this goal.
    """
    return create_clustering_goal(
        in_children="dummy",
        out_children="dummy",
        out_parents="dummy",
        duration=duration,
        min_cluster_size=min_cluster_size,
        min_samples=min_samples,
    )

# AUV4 Limit Defaults from the current params_auv4.yaml in controls
MAX_XY_VEL = 0.5
MAX_XY_ACC = 1.0
MAX_XY_JERK = 1.5
MAX_Z_VEL = 0.2
MAX_Z_ACC = 1.0
MAX_Z_JERK = 1.5
MAX_YAW_VEL = 0.3
MAX_YAW_ACC = 0.5
MAX_YAW_JERK = 0.5

def create_limits_srv_request(
    max_xy_vel=MAX_XY_VEL,
    max_xy_acc=MAX_XY_ACC,
    max_xy_jerk=MAX_XY_JERK,
    max_z_vel=MAX_Z_VEL,
    max_z_acc=MAX_Z_ACC,
    max_z_jerk=MAX_Z_JERK,
    max_yaw_vel=MAX_YAW_VEL,
    max_yaw_acc=MAX_YAW_ACC,
    max_yaw_jerk=MAX_YAW_JERK,
):

    """
    Generates a Limit request to be sent over to controls. Only need
    to define what you want changed. An empty call will return a request with default values
    from params_auv4.yaml
    """

    request = Limits.Request()
    request.max_xy_vel = max_xy_vel
    request.max_xy_acc = max_xy_acc
    request.max_xy_jerk = max_xy_jerk
    request.max_z_vel = max_z_vel
    request.max_z_acc = max_z_acc    
    request.max_z_jerk = max_z_jerk
    request.max_yaw_vel = max_yaw_vel
    request.max_yaw_acc = max_yaw_acc
    request.max_yaw_jerk = max_yaw_jerk

    return request

