import py_trees
import py_trees_ros
from rclpy.qos import qos_profile_system_default

from mission_planner_2.commons import cache_tf


def _tf_fallback_check(
    frame_id: str,
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

    set_missing_tf = py_trees.behaviours.SetBlackboardVariable(
        name="set_missing_tf",
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


def create_tf_checker_root(
    start_frames: list[str] = ["_placeholder1", "_placeholder2"],
    end_frames: list[str] = ["_placeholder1u", "_placeholder2u"],
    timeout: float = 5.0,
    update_keys: list[str] = ["_placeholder1u", "_placeholder2u"],
    fallback_val: list[any] = [None, None],
    is_from_bb: bool = False,
) -> py_trees.composites.Sequence:
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
            is_from_bb=is_from_bb,
        )
        for start, end, update_key, fallback_val in zip(
            start_frames, end_frames, update_keys, fallback_val
        )
    ]

    root.add_children(tf_checker_list)

    return root
