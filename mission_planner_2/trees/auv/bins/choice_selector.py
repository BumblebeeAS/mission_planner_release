import operator

import py_trees

from mission_planner_2.commons.pose_utils import create_stamped_pose


def create_choice_selector_root(
    choice_key: str = "choice",
    pose_key: str = "pose",
    fish_bin_frame: str = "bin/fish",
    shark_bin_frame: str = "bin/shark",
) -> py_trees.composites.Selector:

    # Define nodes in execution order
    # Root selector - will try fish sequence first, then shark as fallback
    sel_choice_selector_root = py_trees.composites.Selector(
        name="Choice selector root", memory=True
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
    set_pose_fish = py_trees.behaviours.SetBlackboardVariable(
        name="Set fish bin pose",
        variable_name=pose_key,
        variable_value=create_stamped_pose(fish_bin_frame),
        overwrite=True,
    )

    # Step 3: Set shark bin pose (fallback option)
    set_pose_shark = py_trees.behaviours.SetBlackboardVariable(
        name="Set shark bin pose",
        variable_name=pose_key,
        variable_value=create_stamped_pose(shark_bin_frame),
        overwrite=True,
    )

    # Build tree structure
    seq_fish_setup.add_children([check_is_fish, set_pose_fish])
    sel_choice_selector_root.add_children([seq_fish_setup, set_pose_shark])

    return sel_choice_selector_root
