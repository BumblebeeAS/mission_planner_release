import py_trees
import py_trees_ros
from rclpy.qos import qos_profile_system_default

from mission_planner_2.commons import cache_tf


def _tf_fallback_check(
    frame_id: str,
    is_fail_if_none: bool,
    end_frame: str = "world_ned",
    timeout: float = 5.0,
    update_key: str = "_placeholder1u",
    fallback_val: any = None,
    is_from_bb: bool = False,
) -> py_trees.composites.Selector:
    root = py_trees.composites.Selector(
        name="tf_fallback",
        memory=True,
    )

    if is_from_bb:
        sub_tf = cache_tf.ToBlackboardFromBlackboard(
            name="sub_tf_from_bb",
            variable_name=update_key,
            target_frame_key=frame_id,
            source_frame_key=end_frame,
            qos_profile=qos_profile_system_default,
        )
    else:
        sub_tf = py_trees_ros.transforms.ToBlackboard(
            name="sub_tf",
            variable_name=update_key,
            target_frame=frame_id,
            source_frame=end_frame,
            qos_profile=qos_profile_system_default,
        )

    decorator_timeout = py_trees.decorators.Timeout(
        name="tf_fallback_timeout",
        child=sub_tf,
        duration=timeout,
    )

    if is_fail_if_none:
        fallback_behavior = py_trees.behaviours.Failure(
            name=f"tf checker fail_if_none: {is_fail_if_none}"
        )
    else:
        fallback_behavior = py_trees.behaviours.SetBlackboardVariable(
            name=f"set_missing_tf fail_if_none: {is_fail_if_none}",
            variable_name=update_key,
            variable_value=fallback_val,
            overwrite=True,
        )

    root.add_children(
        [
            decorator_timeout,
            set_missing_tf,
        ]
    )

    return root


def create_tf_checker_from_constant_root(
    start_frames: list[str] = ["_placeholder1", "_placeholder2"],
    end_frames: list[str] = ["_placeholder1u", "_placeholder2u"],
    timeout: float = 5.0,
    update_keys: list[str] = ["_placeholder1u", "_placeholder2u"],
    fallback_val: list[any] = [None, None],
    is_fail_if_none: bool = True,
) -> py_trees.composites.Sequence:
    """Creates a sequence of TF checks with constant frames.

    Args:
        start_frames (list[str], optional): list of start frame ids. Defaults to ["_placeholder1", "_placeholder2"].
        end_frames (list[str], optional): list of end frame ids. Defaults to ["_placeholder1u", "_placeholder2u"].
        timeout (float, optional): timeout for the tf lookup. Defaults to 5.0.
        update_keys (list[str], optional): list of keys that will be used to store the TransformStamped msg. Defaults to ["_placeholder1u", "_placeholder2u"].
        fallback_val (list[any], optional): value that will be set should the tf lookup fail. Defaults to [None, None].

    Returns:
        py_trees.composites.Sequence: A sequence of TF lookups that will be executed in order.
    """
    root = py_trees.composites.Sequence(
        name="tf_checker sequence",
        memory=True,
    )

    tf_checker_list = [
        _tf_fallback_check(
            frame_id=start,
            end_frame=end,
            timeout=timeout,
            update_key=update_key,
            fallback_val=fallback_val,
            is_from_bb=False,
            is_fail_if_none=is_fail_if_none,
        )
        for start, end, update_key, fallback_val in zip(
            start_frames, end_frames, update_keys, fallback_val
        )
    ]

    root.add_children(tf_checker_list)

    return root


def create_tf_checker_from_bb_root(
    start_frame_keys: list[str] = ["_placeholder1", "_placeholder2"],
    end_frame_keys: list[str] = ["_placeholder1u", "_placeholder2u"],
    timeout: float = 5.0,
    update_keys: list[str] = ["_placeholder1u", "_placeholder2u"],
    fallback_val: list[any] = [None, None],
    is_fail_if_none: bool = True,
) -> py_trees.composites.Sequence:
    """Creates a sequence of TF checks from blackboard.

    Args:
        start_frame_keys (list[str], optional): blackboard keys that map to the start_frame. Defaults to ["_placeholder1", "_placeholder2"].
        end_frame_keys (list[str], optional): blackboard keys that map to the end frame. Defaults to ["_placeholder1u", "_placeholder2u"].
        timeout (float, optional): timeout for tf lookup. Defaults to 5.0.
        update_keys (list[str], optional): blackboard keys to store the TransformStamped msg. Defaults to ["_placeholder1u", "_placeholder2u"].
        fallback_val (list[any], optional): value that will be set should the tf lookup fail. Defaults to [None, None].

    Returns:
        py_trees.composites.Sequence: A sequence of TF lookups that will be executed in order.
    """
    root = py_trees.composites.Sequence(
        name="tf_checker sequence from bb",
        memory=True,
    )

    tf_checker_list = [
        _tf_fallback_check(
            frame_id=start,
            end_frame=end,
            timeout=timeout,
            update_key=update_key,
            fallback_val=fallback_val,
            is_from_bb=True,
            is_fail_if_none=is_fail_if_none,
        )
        for start, end, update_key, fallback_val in zip(
            start_frame_keys, end_frame_keys, update_keys, fallback_val
        )
    ]

    root.add_children(tf_checker_list)

    return root
