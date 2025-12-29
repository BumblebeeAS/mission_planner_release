import operator
import uuid
from collections.abc import Callable, Sequence

import py_trees
from geometry_msgs.msg import TransformStamped

from mission_planner_2.vehicles.shared.trees.blackboard import DynamicSetBlackboard
from mission_planner_2.vehicles.shared.trees.tf_checker import (
    create_tf_checker_from_bb_root,
    create_tf_checker_from_constant_root,
)


def _default_func(tf):
    raise NotImplementedError("Please provide a function to check within threshold")


def _gen_namespace():
    """Generate a unique namespace for the cluster and goto nodes."""
    unique_id = uuid.uuid4()
    return f"/cluster_goto_{unique_id}"


def _create_internal_keys(num_tfs: int = 2):
    """Create internal keys for the cluster and goto nodes."""
    namespace = _gen_namespace()
    tf_keys = [f"{namespace}/tf_{i}" for i in range(num_tfs)]
    check_res_key = f"{namespace}/is_within_threshold"
    return tf_keys, check_res_key


def _create_goto_cluster_retry(
    cluster_node_check: py_trees.behaviour.Behaviour,
    goto_node: py_trees.behaviour.Behaviour,
    tf_checker: py_trees.behaviour.Behaviour,
    tf_keys: list[str],
    check_res_key: str,
    within_threshold_list: list[Callable[[TransformStamped], bool]],
    retries: int,
    stabilization_duration: float,
):
    wait = py_trees.timers.Timer(
        name="Wait for controls to stabilize", duration=stabilization_duration
    )

    set_threshold_check = DynamicSetBlackboard(
        name="Apply threshold check",
        key=tf_keys,
        update_key=check_res_key,
        func=lambda *args: all(
            threshold_func(tf)
            for tf, threshold_func in zip(args, within_threshold_list)
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
            # wait,
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
    cluster_node: py_trees.behaviour.Behaviour,
    cluster_node_check: py_trees.behaviour.Behaviour,
    goto_node: py_trees.behaviour.Behaviour,
    retries: int = 3,
    start_frame_keys: list[str] = ["key_start_frame"],
    goto_pose_frame_key: str = "key_goto_pose",
    stabilization_duration: float = 5.0,
    name="cluster_and_goto",
    within_threshold_list: Sequence[Callable[[TransformStamped], bool]] = [
        _default_func
    ],
):
    root = py_trees.composites.Sequence(
        name=name,
        memory=True,
    )

    if len(within_threshold_list) != len(start_frame_keys):
        raise ValueError(
            "The length of within_threshold_list must match the number of start_frame_keys."
        )
    num = len(within_threshold_list)

    tf_keys, check_res_key = _create_internal_keys(num)

    # TODO: if want to expose the timeout
    tf_checker_bb = create_tf_checker_from_bb_root(
        # start_frame_keys=[anchor_frame_key, "/global/base_link"],
        start_frame_keys=start_frame_keys,
        end_frame_keys=[goto_pose_frame_key] * num,
        update_keys=tf_keys,
        fallback_val=[None] * num,
    )

    retry = _create_goto_cluster_retry(
        cluster_node_check=cluster_node_check,
        goto_node=goto_node,
        tf_checker=tf_checker_bb,
        retries=retries,
        stabilization_duration=stabilization_duration,
        within_threshold_list=within_threshold_list,
        tf_keys=tf_keys,
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
    cluster_node: py_trees.behaviour.Behaviour,
    cluster_node_check: py_trees.behaviour.Behaviour,
    goto_node: py_trees.behaviour.Behaviour,
    retries: int = 3,
    start_frame_keys: list[str] = ["auv4/base_link_ned"],
    goto_pose_frame: str = "auv4/base_link_ned",
    stabilization_duration: float = 5.0,
    name: str = "cluster_and_goto",
    within_threshold_list: Sequence[Callable[[TransformStamped], bool]] = [
        _default_func
    ],
):
    root = py_trees.composites.Sequence(
        name=name,
        memory=True,
    )
    if len(within_threshold_list) != len(start_frame_keys):
        raise ValueError(
            "The length of within_threshold_list must match the number of start_frame_keys."
        )

    num = len(within_threshold_list)

    tf_keys, check_res_key = _create_internal_keys(num)

    tf_checker = create_tf_checker_from_constant_root(
        # start_frames=[anchor_frame, "auv4/base_link_ned"],
        start_frames=start_frame_keys,
        end_frames=[goto_pose_frame] * num,
        update_keys=tf_keys,
        fallback_val=[None] * num,
    )

    retry = _create_goto_cluster_retry(
        cluster_node_check=cluster_node_check,
        goto_node=goto_node,
        tf_checker=tf_checker,
        retries=retries,
        stabilization_duration=stabilization_duration,
        within_threshold_list=within_threshold_list,
        tf_keys=tf_keys,
        check_res_key=check_res_key,
    )

    root.add_children(
        [
            cluster_node,
            retry,
        ]
    )

    return root


def _create_goto_cluster_tf_tf_root(
    cluster_node: py_trees.behaviour.Behaviour,
    cluster_node_check: py_trees.behaviour.Behaviour,
    goto_node: py_trees.behaviour.Behaviour,
    tf_checker_1: py_trees.behaviour.Behaviour,
    tf_checker_2: py_trees.behaviour.Behaviour,
    tf_1_key: str,
    tf_2_key: str,
    check_res_key: str,
    retries: int,
    stabilization_duration: float,
    distance_threshold: float,
    within_threshold: Callable,
    name: str = "cluster_and_goto_tf_tf",
):
    root = py_trees.composites.Sequence(
        name=name,
        memory=True,
    )

    wait = py_trees.timers.Timer(
        name="Wait for controls to stabilize", duration=stabilization_duration
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
            tf_checker_1,
            goto_node,
            # wait,
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


def create_goto_cluster_from_bb_tf_tf_root(
    cluster_node: py_trees.behaviour.Behaviour,
    cluster_node_check: py_trees.behaviour.Behaviour,
    goto_node: py_trees.behaviour.Behaviour,
    distance_threshold: float | None = None,
    retries: int = 3,
    tf_frame_key: str = "something/clustered",
    stabilization_duration: float = 5.0,
    name="cluster_and_goto_tf_tf",
    within_threshold=lambda x, y: NotImplementedError(
        "Please provide a function to check within threshold"
    ),
):
    # FIXME @wesley
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

    cluster_retry = _create_goto_cluster_tf_tf_root(
        cluster_node=cluster_node,
        cluster_node_check=cluster_node_check,
        goto_node=goto_node,
        tf_checker_1=tf_checker,
        tf_checker_2=tf_checker_2,
        distance_threshold=distance_threshold,
        retries=retries,
        stabilization_duration=stabilization_duration,
        name=name,
        within_threshold=within_threshold,
        tf_1_key=tf_1_key,
        tf_2_key=tf_2_key,
        check_res_key=check_res_key,
    )

    return cluster_retry


def create_goto_cluster_from_constant_tf_tf_root(
    cluster_node: py_trees.behaviour.Behaviour,
    cluster_node_check: py_trees.behaviour.Behaviour,
    goto_node: py_trees.behaviour.Behaviour,
    distance_threshold: float | None = None,
    retries: int = 3,
    tf_frame: str = "something/clustered",
    stabilization_duration: float = 5.0,
    name="cluster_and_goto_tf_tf",
    within_threshold=lambda x, y: NotImplementedError(
        "Please provide a function to check within threshold"
    ),
):
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

    cluster_retry = _create_goto_cluster_tf_tf_root(
        cluster_node=cluster_node,
        cluster_node_check=cluster_node_check,
        goto_node=goto_node,
        tf_checker_1=tf_checker,
        tf_checker_2=tf_checker_2,
        distance_threshold=distance_threshold,
        retries=retries,
        stabilization_duration=stabilization_duration,
        name=name,
        within_threshold=within_threshold,
        tf_1_key=tf_1_key,
        tf_2_key=tf_2_key,
        check_res_key=check_res_key,
    )

    return cluster_retry
