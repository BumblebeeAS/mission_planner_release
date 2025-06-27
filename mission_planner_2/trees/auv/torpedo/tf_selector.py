import operator

import py_trees

from mission_planner_2.commons import cache_tf
from mission_planner_2.commons.pose_utils import create_stamped_pose


def create_tf_selector_root(
    choice_key: str = "choice",
    pose_key: str = "pose",
    go_back_pose_key: str = "go_back_pose",
    is_first: bool = True,
    fish_shoot_frame: str = "torpedo_1/fish/shoot",
    shark_shoot_frame: str = "torpedo_1/shark/shoot",
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
    sel_tf_root = py_trees.composites.Selector(
        name="Tf selector root",
        memory=True,
    )

    seq_fish_setup = py_trees.composites.Sequence(
        name="Fish setup seq",
        memory=True,
    )

    seq_shark_setup = py_trees.composites.Sequence(
        name="Shark setup seq",
        memory=True,
    )

    # xor(1, 1)-> false xor(0, 1) -> true
    check_is_fish = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check is fish",
        check=py_trees.common.ComparisonExpression(
            variable=choice_key,
            value=False,
            operator=lambda x, y: operator.eq(x.success ^ is_first, y),
        ),
    )

    cache_tf_fish = cache_tf.ToBlackboard(
        name="Cache tf fish",
        variable_name=go_back_pose_key,
        start=fish_shoot_frame,
        end="auv4/base_link_ned",
    )

    cache_tf_shark = cache_tf.ToBlackboard(
        name="Cache tf shark",
        variable_name=go_back_pose_key,
        start=shark_shoot_frame,
        end="auv4/base_link_ned",
    )

    set_pose_fish = py_trees.behaviours.SetBlackboardVariable(
        name="Set hole target pose",
        variable_name=pose_key,
        variable_value=create_stamped_pose(fish_shoot_frame),
        overwrite=True,
    )

    set_pose_shark = py_trees.behaviours.SetBlackboardVariable(
        name="Set hole target pose",
        variable_name=pose_key,
        variable_value=create_stamped_pose(shark_shoot_frame),
        overwrite=True,
    )

    seq_fish_setup.add_children([check_is_fish, cache_tf_fish, set_pose_fish])
    seq_shark_setup.add_children([cache_tf_shark, set_pose_shark])
    sel_tf_root.add_children([seq_fish_setup, seq_shark_setup])

    return sel_tf_root
