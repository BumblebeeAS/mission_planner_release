import operator

import numpy as np
import py_trees

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.tf_checker import create_tf_checker_root


def create_goto_cluster_root(
    cluster_node: py_trees.behaviour,
    cluster_node_check: py_trees.behaviour,
    goto_node: py_trees.behaviour,
    distance_threshold: float | None = None,
    yaw_threshold: float | None = None,
    retries: int = 3,
    anchor_frame: str = "auv4/base_link_ned",
    goto_pose_frame_id: str = "auv4/base_link_ned",
    stabilization_duration: float = 5.0,
    name="cluster_and_goto",
    within_threshold=lambda x, y: NotImplementedError(
        "Please provide a function to check within threshold"
    ),
    is_from_bb: bool = False,
):
    xyz_tf_key = "xyz_tf"
    rpy_tf_key = "rpy_tf"
    check_res_key = "is_within_threshold"

    if distance_threshold is None:
        distance_threshold = np.inf

    if yaw_threshold is None:
        yaw_threshold = 370.0

    wait = py_trees.timers.Timer(
        name="Wait between clusters", duration=stabilization_duration
    )

    tf_checker = create_tf_checker_root(
        start_frames=[anchor_frame, "auv4/base_link_ned"],
        end_frames=[goto_pose_frame_id, goto_pose_frame_id],
        update_keys=[xyz_tf_key, rpy_tf_key],
        fallback_val=[None, None],
        is_from_bb=is_from_bb,
    )

    set_threshold_check = DynamicSetBlackboard(
        name="Apply threshold check",
        key=[xyz_tf_key, rpy_tf_key],
        update_key=check_res_key,
        func=lambda xyz_pose, rpy_pose: within_threshold(
            xyz_pose, rpy_pose, distance_threshold, yaw_threshold
        ),
    )

    check_within_threshold = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify within threshold",
        check=py_trees.common.ComparisonExpression(
            variable=check_res_key,
            value=True,
            operator=operator.eq,
        ),
    )

    retry_seq = py_trees.composites.Sequence(
        "Cluster and move sequence",
        memory=True,
        children=[
            goto_node,
            wait,
            cluster_node_check,
            tf_checker,
            set_threshold_check,
            check_within_threshold,
        ],
    )

    retry = py_trees.decorators.Retry(
        name="Retry",
        child=retry_seq,
        num_failures=retries,
    )

    root = py_trees.composites.Sequence(
        name=name,
        memory=True,
        children=[
            cluster_node,
            retry,
        ],
    )

    return root
