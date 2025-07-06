import operator
import uuid

import numpy as np
import py_trees
from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.tf_checker import (
    create_tf_checker_from_bb_root,
    create_tf_checker_from_constant_root,
)


def _gen_namespace():
    """Generate a unique namespace for the cluster and goto nodes."""
    unique_id = uuid.uuid4()
    return f"/cluster_goto_{unique_id}"


def _create_internal_keys():
    """Create internal keys for the cluster and goto nodes."""
    namespace = _gen_namespace()
    tf_1_key = f"{namespace}/xyz_tf"
    tf_2_key = f"{namespace}/rpy_tf"
    check_res_key = f"{namespace}/is_within_threshold"
    return tf_1_key, tf_2_key, check_res_key


def _create_goto_cluster_retry(
    cluster_node_check: py_trees.behaviour,
    goto_node: py_trees.behaviour,
    tf_checker: py_trees.behaviour,
    distance_threshold: float | None = None,
    yaw_threshold: float | None = None,
    retries: int = 3,
    stabilization_duration: float = 5.0,
    within_threshold=lambda x, y: NotImplementedError(
        "Please provide a function to check within threshold"
    ),
    tf_1_key: str | None = None,
    tf_2_key: str | None = None,
    check_res_key: str | None = None,
):
    if distance_threshold is None:
        distance_threshold = np.inf

    if yaw_threshold is None:
        yaw_threshold = 370.0

    wait = py_trees.timers.Timer(
        name="Wait between clusters", duration=stabilization_duration
    )

    set_threshold_check = DynamicSetBlackboard(
        name="Apply threshold check",
        key=[tf_1_key, tf_2_key],
        update_key=check_res_key,
        func=lambda tf1, tf2: within_threshold(
            tf1, tf2, distance_threshold, yaw_threshold
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

    seq_retry = py_trees.composites.Sequence(
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
        child=seq_retry,
        num_failures=retries,
    )

    return retry


def create_goto_cluster_from_bb_root(
    cluster_node: py_trees.behaviour,
    cluster_node_check: py_trees.behaviour,
    goto_node: py_trees.behaviour,
    distance_threshold: float | None = None,
    yaw_threshold: float | None = None,
    retries: int = 3,
    anchor_frame_key: str = "key_anchor",
    goto_pose_frame_key: str = "key_goto_pose",
    stabilization_duration: float = 5.0,
    name="cluster_and_goto",
    within_threshold=lambda x, y: NotImplementedError(
        "Please provide a function to check within threshold"
    ),
):
    root = py_trees.composites.Sequence(
        name=name,
        memory=True,
    )

    xyz_tf_key, rpy_tf_key, check_res_key = _create_internal_keys()

    # TODO: if want to expose the timeout
    tf_checker_bb = create_tf_checker_from_bb_root(
        start_frame_keys=[anchor_frame_key, "/global/base_link"],
        end_frame_keys=[goto_pose_frame_key, goto_pose_frame_key],
        update_keys=[xyz_tf_key, rpy_tf_key],
        fallback_val=[None, None],
    )

    retry = _create_goto_cluster_retry(
        cluster_node_check=cluster_node_check,
        goto_node=goto_node,
        tf_checker=tf_checker_bb,
        distance_threshold=distance_threshold,
        yaw_threshold=yaw_threshold,
        retries=retries,
        stabilization_duration=stabilization_duration,
        within_threshold=within_threshold,
        tf_1_key=xyz_tf_key,
        tf_2_key=rpy_tf_key,
        check_res_key=check_res_key,
    )

    root.add_children(
        [
            cluster_node,
            retry,
        ]
    )

    return root


def create_goto_cluster_from_constant_root(
    cluster_node: py_trees.behaviour,
    cluster_node_check: py_trees.behaviour,
    goto_node: py_trees.behaviour,
    distance_threshold: float | None = None,
    yaw_threshold: float | None = None,
    retries: int = 3,
    anchor_frame: str = "auv4/base_link_ned",
    goto_pose_frame: str = "auv4/base_link_ned",
    stabilization_duration: float = 5.0,
    name="cluster_and_goto",
    within_threshold=lambda x, y: NotImplementedError(
        "Please provide a function to check within threshold"
    ),
):
    root = py_trees.composites.Sequence(
        name=name,
        memory=True,
    )

    xyz_tf_key, rpy_tf_key, check_res_key = _create_internal_keys()

    tf_checker = create_tf_checker_from_constant_root(
        start_frames=[anchor_frame, "auv4/base_link_ned"],
        end_frames=[goto_pose_frame, goto_pose_frame],
        update_keys=[xyz_tf_key, rpy_tf_key],
        fallback_val=[None, None],
    )

    retry = _create_goto_cluster_retry(
        cluster_node_check=cluster_node_check,
        goto_node=goto_node,
        tf_checker=tf_checker,
        distance_threshold=distance_threshold,
        yaw_threshold=yaw_threshold,
        retries=retries,
        stabilization_duration=stabilization_duration,
        within_threshold=within_threshold,
        tf_1_key=xyz_tf_key,
        tf_2_key=rpy_tf_key,
        check_res_key=check_res_key,
    )

    root.add_children(
        [
            cluster_node,
            retry,
        ]
    )

    return root


def create_goto_cluster_from_bb_tf_tf_root(
    cluster_node: py_trees.behaviour,
    cluster_node_check: py_trees.behaviour,
    goto_node: py_trees.behaviour,
    distance_threshold: float | None = None,
    retries: int = 3,
    tf_frame_key: str = "something/clustered",
    stabilization_duration: float = 5.0,
    name="cluster_and_goto_tf_tf",
    within_threshold=lambda x, y: NotImplementedError(
        "Please provide a function to check within threshold"
    ),
):
    root = py_trees.composites.Sequence(
        name=name,
        memory=True,
    )

    tf_1_key, tf_2_key, check_res_key = _create_internal_keys()

    tf_checker = create_tf_checker_from_bb_root(
        start_frame_keys=[tf_frame_key],
        end_frame_keys=["/global/world"],
        update_keys=[tf_1_key],
        fallback_val=[None],
    )

    tf_checker_2 = create_tf_checker_from_bb_root(
        start_frame_keys=[tf_frame_key],
        end_frame_keys=["/global/world"],
        update_keys=[tf_2_key],
        fallback_val=[None],
    )

    wait = py_trees.timers.Timer(
        name="Wait between clusters", duration=stabilization_duration
    )

    set_threshold_check = DynamicSetBlackboard(
        name="Apply threshold check",
        key=[tf_1_key, tf_2_key],
        update_key=check_res_key,
        func=lambda tf1, tf2: within_threshold(tf1, tf2, distance_threshold),
    )

    check_within_threshold = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify within threshold",
        check=py_trees.common.ComparisonExpression(
            variable=check_res_key,
            value=True,
            operator=operator.eq,
        ),
    )

    root.add_children(
        [
            cluster_node,
            tf_checker,
            goto_node,
            wait,
            cluster_node_check,
            tf_checker_2,
            set_threshold_check,
            check_within_threshold,
        ]
    )

    retry = py_trees.decorators.Retry(
        name="Retry",
        child=root,
        num_failures=retries,
    )

    return retry


def create_goto_cluster_from_constant_tf_tf_root(
    cluster_node: py_trees.behaviour,
    cluster_node_check: py_trees.behaviour,
    goto_node: py_trees.behaviour,
    distance_threshold: float | None = None,
    retries: int = 3,
    tf_frame: str = "something/clustered",
    stabilization_duration: float = 5.0,
    name="cluster_and_goto_tf_tf",
    within_threshold=lambda x, y: NotImplementedError(
        "Please provide a function to check within threshold"
    ),
):
    root = py_trees.composites.Sequence(
        name=name,
        memory=True,
    )

    tf_1_key, tf_2_key, check_res_key = _create_internal_keys()

    tf_checker = create_tf_checker_from_constant_root(
        start_frames=[tf_frame],
        end_frames=["world_ned"],
        update_keys=[tf_1_key],
        fallback_val=[None],
    )

    tf_checker_2 = create_tf_checker_from_constant_root(
        start_frames=[tf_frame],
        end_frames=["world_ned"],
        update_keys=[tf_2_key],
        fallback_val=[None],
    )

    wait = py_trees.timers.Timer(
        name="Wait between clusters", duration=stabilization_duration
    )

    set_threshold_check = DynamicSetBlackboard(
        name="Apply threshold check",
        key=[tf_1_key, tf_2_key],
        update_key=check_res_key,
        func=lambda tf1, tf2: within_threshold(tf1, tf2, distance_threshold),
    )

    check_within_threshold = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify within threshold",
        check=py_trees.common.ComparisonExpression(
            variable=check_res_key,
            value=True,
            operator=operator.eq,
        ),
    )

    root.add_children(
        [
            cluster_node,
            tf_checker,
            goto_node,
            wait,
            cluster_node_check,
            tf_checker_2,
            set_threshold_check,
            check_within_threshold,
        ]
    )

    retry = py_trees.decorators.Retry(
        name="Retry",
        child=root,
        num_failures=retries,
    )

    return retry
