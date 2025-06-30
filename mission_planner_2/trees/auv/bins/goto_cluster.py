import operator

import numpy as np
import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from geometry_msgs.msg import PoseStamped
from py_trees_ros.action_clients import uuid
from tf_transformations import euler_from_quaternion

from mission_planner_2.commons import cache_tf
from mission_planner_2.commons.blackboard import (
    DynamicSetBlackboard,
    convert_to_safe_name,
)
from mission_planner_2.trees.auv.goto import goto


def FromBlackboard(
    clustering_goal_key: str,
    goto_pose_key: str,
    distance_threshold: float | None = None,
    yaw_threshold: float | None = None,
    retries: int = 3,
    anchor_frame: str = "auv4/base_link_ned",
    specified_heading=True,
    ignore_depth=False,
    stabilization_duration: float = 5.0,
    name="cluster_and_goto",
):
    client = py_trees.blackboard.Client(namespace="/")
    clustering_goal = client.get(clustering_goal_key)
    goto_pose = client.get(goto_pose_key)

    return FromConstant(
        clustering_goal=clustering_goal,
        goto_pose=goto_pose,
        distance_threshold=distance_threshold,
        yaw_threshold=yaw_threshold,
        retries=retries,
        anchor_frame=anchor_frame,
        specified_heading=specified_heading,
        ignore_depth=ignore_depth,
        stabilization_duration=stabilization_duration,
        name=name,
    )


def FromConstant(
    clustering_goal: ClusterTf.Goal,
    goto_pose: PoseStamped,
    distance_threshold: float | None = None,
    yaw_threshold: float | None = None,
    retries: int = 3,
    anchor_frame: str = "auv4/base_link_ned",
    specified_heading=True,
    ignore_depth=False,
    stabilization_duration: float = 5.0,
    name="cluster_and_goto",
):
    xyz_tf_key = py_trees.blackboard.Blackboard.absolute_name(
        "/", convert_to_safe_name(name) + "/" + str(uuid.uuid4()).replace("-", "")
    )

    rpy_tf_key = py_trees.blackboard.Blackboard.absolute_name(
        "/", convert_to_safe_name(name) + "/" + str(uuid.uuid4()).replace("-", "")
    )

    check_key = py_trees.blackboard.Blackboard.absolute_name(
        "/", convert_to_safe_name(name) + "/" + str(uuid.uuid4()).replace("-", "")
    )

    if distance_threshold is None:
        distance_threshold = np.inf

    if yaw_threshold is None:
        yaw_threshold = 370.0

    action_cluster = py_trees_ros.action_clients.FromConstant(
        name="Cluster transforms",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=clustering_goal,
    )

    goto_location = goto.FromConstant(
        name="Goto location",
        pose=goto_pose,
        anchor_frame_name=anchor_frame,
        specified_heading=specified_heading,
        ignore_depth=ignore_depth,
    )

    wait = py_trees.timers.Timer(
        name="Wait between clusters", duration=stabilization_duration
    )

    extract_xyz_tf = cache_tf.ToBlackboard(
        name="Cache xyz",
        variable_name=xyz_tf_key,
        start=anchor_frame,
        end=goto_pose.header.frame_id,
    )

    extract_rpy_tf = cache_tf.ToBlackboard(
        name="Cache rpy",
        variable_name=rpy_tf_key,
        start="auv4/base_link_ned",
        end=goto_pose.header.frame_id,
    )

    threshold_check = DynamicSetBlackboard(
        name="Apply threshold check",
        key=[xyz_tf_key, rpy_tf_key],
        update_key=check_key,
        func=lambda xyz_pose, rpy_pose: within_threshold(
            xyz_pose, rpy_pose, distance_threshold, yaw_threshold
        ),
    )

    check_within_threshold = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify within threshold",
        check=py_trees.common.ComparisonExpression(
            variable=check_key,
            value=True,
            operator=operator.eq,
        ),
    )

    main_seq = py_trees.composites.Sequence(
        "Cluster and move sequence",
        memory=True,
        children=[
            action_cluster,
            goto_location,
            wait,
            extract_xyz_tf,
            extract_rpy_tf,
            threshold_check,
            check_within_threshold,
        ],
    )

    root = py_trees.decorators.Retry(
        name=name,
        child=main_seq,
        num_failures=retries,
    )

    return root


def within_threshold(
    xyz_pose: PoseStamped,
    rpy_pose: PoseStamped,
    distance_threshold: float,
    yaw_threshold: float,
) -> bool:
    _, _, y = euler_from_quaternion(
        [
            rpy_pose.pose.orientation.x,
            rpy_pose.pose.orientation.y,
            rpy_pose.pose.orientation.z,
            rpy_pose.pose.orientation.w,
        ]
    )

    y = np.degrees(y) % 360

    if y > yaw_threshold:
        return False

    distance = np.sqrt(
        (xyz_pose.pose.position.x**2)
        + (xyz_pose.pose.position.y**2)
        + (xyz_pose.pose.position.z**2)
    )

    if distance > distance_threshold:
        return False

    return True
