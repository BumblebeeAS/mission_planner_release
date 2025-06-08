import operator

import py_trees

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.pose_utils import create_stamped_pose


def create_bin_selector_root(
    choice_key: str = "choice",
    pose_key: str = "pose",
    points1_key: str = "points_1",
    points2_key: str = "points_2",
    fish_bin_frame: str = "bin/fish",
    shark_bin_frame: str = "bin/shark",
    fish_bin_rotated_frame: str = "bin/fish/rotated",
    shark_bin_rotated_frame: str = "bin/shark/rotated",
) -> py_trees.composites.Selector:
    """
    Creates the root node of the bin selector tree.
    This tree merely checks the choice that was made and set the pose accordingly. The "pose"
    in this case is merely the origin of the target/frame of interest. Transform between the frame of interest
    and vision output should be defined in cfg.yaml.
    Args:
        choice_key (str, optional): Full key of the choice blackboard variable. Defaults to "choice".
        pose_key (str, optional): Full key of the pose blackboard variable. Defaults to "pose".
        points1_key (str, optional): Full key of the first points blackboard variable. Defaults to "points_1".
        points2_key (str, optional): Full key of the second points blackboard variable. Defaults to "points_2".
    Returns:
        py_trees.composites.Selector: Root node of the bin selector tree
    """

    # Define nodes in execution order
    # Root selector - will try fish sequence first, then shark as fallback
    sel_bin_selector_root = py_trees.composites.Selector(
        name="Bin selector root", memory=True
    )

    # Fish sequence - executes if choice is fish
    seq_fish_setup = py_trees.composites.Sequence(name="Fish setup", memory=True)

    # Step 1: Check if choice is fish
    check_is_fish = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check if fish",
        check=py_trees.common.ComparisonExpression(
            variable=choice_key,
            value=True,
            operator=lambda x, y: operator.eq(x.success, y),
        ),
    )

    # Step 2: Set fish bin pose
    set_pose_fish = DynamicSetBlackboard(
        name="Set fish bin pose (dynamic)",
        key=[points1_key, points2_key],
        update_key=pose_key,
        overwrite=True,
        func=lambda points1, points2: (
            create_stamped_pose(fish_bin_frame)
            if len(points1.data) > len(points2.data)
            else create_stamped_pose(fish_bin_rotated_frame)
        ),
    )

    # Shark sequence - executes as fallback (always succeeds)
    seq_shark_setup = py_trees.composites.Sequence(name="Shark setup", memory=True)

    # Step 3: Set shark bin pose (fallback option)
    set_pose_shark = DynamicSetBlackboard(
        name="Set shark bin pose (dynamic)",
        key=[points1_key, points2_key],
        update_key=pose_key,
        overwrite=True,
        func=lambda points1, points2: (
            create_stamped_pose(shark_bin_frame)
            if len(points1.data) > len(points2.data)
            else create_stamped_pose(shark_bin_rotated_frame)
        ),
    )

    # Build tree structure
    seq_fish_setup.add_children([check_is_fish, set_pose_fish])
    seq_shark_setup.add_child(set_pose_shark)
    sel_bin_selector_root.add_children([seq_fish_setup, seq_shark_setup])

    return sel_bin_selector_root
