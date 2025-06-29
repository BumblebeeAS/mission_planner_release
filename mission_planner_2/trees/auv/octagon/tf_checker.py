import py_trees
import py_trees_ros
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger

# Generate namespace automatically from file path DONT set manually
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)


def _tf_fallback_check(
    frame_id: str,
    timeout: float = 5.0,
    update_key: str = "_placeholder1u",
    fallback_val: any = None,
) -> py_trees.composites.Selector:
    root = py_trees.composites.Selector(
        name="tf_fallback",
        memory=True,
    )

    sub_tf = py_trees_ros.transforms.ToBlackboard(
        name="sub_tf",
        variable_name=update_key,
        target_frame=frame_id,
        source_frame="world_ned",
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
    frames: list[str] = ["_placeholder1", "_placeholder2"],
    timeout: float = 5.0,
    update_keys: list[str] = ["_placeholder1u", "_placeholder2u"],
    fallback_val: list[any] = [None, None],
) -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(
        name="tf_checker sequence",
        memory=True,
    )

    tf_checker_list = [
        _tf_fallback_check(
            frame_id=frame,
            timeout=timeout,
            update_key=update_key,
            fallback_val=fallback_val,
        )
        for frame, update_key, fallback_val in zip(frames, update_keys, fallback_val)
    ]

    root.add_children(tf_checker_list)

    return root
