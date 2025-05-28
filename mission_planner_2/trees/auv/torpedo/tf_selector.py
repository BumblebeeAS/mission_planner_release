import operator

import py_trees

from mission_planner_2.commons import cache_tf
from mission_planner_2.commons.pose_utils import create_stamped_pose

FISH_HOLE = "auv4/torpedo_2/fish"
SHARK_HOLE = "auv4/torpedo_2/shark"


def create_tf_selector_root(
    choice_key: str = "choice",
    pose_key: str = "pose",
    go_back_pose_key: str = "go_back_pose",
    is_first: bool = True,
) -> py_trees.composites.Selector:
    """
    Create the root node of the TF selector tree.

    Args:
        choice_key (str, optional): The full key for the choice blackboard variable. Defaults to "choice".
        pose_key (str, optional): The full key for the pose to fire at blackboard variable. Defaults to "pose".
        go_back_pose_key (str, optional): The full key for the go back pose blackboard variable. Defaults to "go_back_pose".
        is_first (bool, optional): Whether this is the first time the selector is being used. Defaults to True.

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
            operator=lambda x, y: operator.eq(x.success ^ is_first, y),
        ),
    )

    save_tf_fish = cache_tf.ToBlackboard(
        name="Save TF fish",
        variable_name=go_back_pose_key,
        start_frame="auv4/torpedo_2/fish",
        end_frame="auv4/base_link_ned",
    )

    save_tf_shark = cache_tf.ToBlackboard(
        name="Save TF Shark",
        variable_name=go_back_pose_key,
        start_frame="auv4/torpedo_2/shark",
        end_frame="auv4/base_link_ned",
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
