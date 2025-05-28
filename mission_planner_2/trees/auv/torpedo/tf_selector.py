import operator

import py_trees
import py_trees_ros
from rclpy.qos import qos_profile_system_default

from mission_planner_2.commons.pose_utils import create_stamped_pose

FISH_HOLE = "auv4/torpedo/fish_hole"  # reference the cfg.yaml to set
SHARK_HOLE = "auv4/torpedo/shark_hole"


def create_tf_selector_root(
    choice_key: str = "choice",
    pose_key: str = "pose",
    reset_tf_key: str = "reset_tf",
    isFirst: bool = True,
) -> py_trees.composites.Selector:
    """
    Create the root node of the TF selector tree.

    Args:
        choice_key (str, optional): The full key for the choice blackboard variable. Defaults to "choice".
        pose_key (str, optional): The full key for the pose to fire at blackboard variable. Defaults to "pose".
        reset_tf_key (str, optional): The full key for the reset TF blackboard variable. Defaults to "reset_tf".
        isFirst (bool, optional): Whether this is the first time the selector is being used. Defaults to True.

    Returns:
        py_trees.composites.Selector: The root node of the TF selector tree.
    """

    # Create the root node of the tree
    root = py_trees.composites.Selector(
        name="TF Selector Root",
        memory=True,
    )

    save_tf_fish_seq = py_trees.composites.Sequence(
        name="Fish seq",
        memory=True,
    )

    save_tf_shark_seq = py_trees.composites.Sequence(
        name="Shark seq",
        memory=True,
    )

    # xor(1, 1)-> false xor(0, 1) -> true
    check_is_fish = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check isFish",
        check=py_trees.common.ComparisonExpression(
            variable=choice_key,
            value=False,
            operator=lambda x, y: operator.eq(x.success ^ isFirst, y),
        ),
    )

    save_tf_fish = py_trees_ros.transforms.ToBlackboard(
        name="Save TF fish",
        variable_name=reset_tf_key,
        target_frame=FISH_HOLE,
        source_frame="auv4/base_link_ned",
        qos_profile=qos_profile_system_default,
    )

    save_tf_shark = py_trees_ros.transforms.ToBlackboard(
        name="Save TF shark",
        variable_name=reset_tf_key,
        target_frame=SHARK_HOLE,
        source_frame="auv4/base_link_ned",
        qos_profile=qos_profile_system_default,
    )

    set_pose_fish = py_trees.behaviours.SetBlackboardVariable(
        name="Set Pose Fish",
        variable_name=pose_key,
        variable_value=create_stamped_pose(FISH_HOLE),
        overwrite=True,
    )

    set_pose_shark = py_trees.behaviours.SetBlackboardVariable(
        name="Set Pose Shark",
        variable_name=pose_key,
        variable_value=create_stamped_pose(SHARK_HOLE),
        overwrite=True,
    )

    save_tf_fish_seq.add_children(
        [
            check_is_fish,
            save_tf_fish,
            set_pose_fish,
        ]
    )

    save_tf_shark_seq.add_children(
        [
            save_tf_shark,
            set_pose_shark,
        ]
    )

    root.add_children(
        [
            save_tf_fish_seq,
            save_tf_shark_seq,
        ]
    )

    return root
