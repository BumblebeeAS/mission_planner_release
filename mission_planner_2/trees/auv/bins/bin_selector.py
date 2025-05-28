import operator

import py_trees

from mission_planner_2.commons.pose_utils import create_stamped_pose

FISH_BIN_FRAME = "auv4/bin/fish"
SHARK_BIN_FRAME = "auv4/bin/shark"

def create_bin_selector_root(
        choice_key: str = "choice",
        pose_key: str = "pose"
) -> py_trees.composites.Selector:
    """
    Creates the root node of the bin selector tree.
    This tree merely checks the choice that was made and set the pose accordingly. The "pose" 
    in this case is merely the origin of the target/frame of interest. Transform between the frame of interest
    and vision output should be defined in cfg.yaml.
    Args:
        choice_key (str, optional): Full key of the choice blackboard variable. Defaults to "choice".
        pose_key (str, optional): Full key of the pose blackboard variable. Defaults to "pose".

    Returns:
        py_trees.composites.Selector: Root node of the bin selector tree
    """
    # Create root node
    bin_selector_root = py_trees.composites.Selector(
        name="bin selector root",
        memory=True
    )

    fish_setup_sequence = py_trees.composites.Sequence(
        name="fish setup sequence",
        memory=True
    )

    shark_setup_sequence = py_trees.composites.Sequence(
        name="shark setup sequence",
        memory=True
    )

    # Check if we received a choice of fish
    check_is_fish = py_trees.behaviours.CheckBlackboardVariableValue(
        name="check is fish",
        check=py_trees.common.ComparisonExpression(
            variable=choice_key,
            value=True,
            operator=lambda x, y: operator.eq(x.success, y)
        )
    )

    # Write desired pose to blackboard
    set_pose_fish = py_trees.behaviours.SetBlackboardVariable(
        name="set bin target pose",
        variable_name=pose_key,
        variable_value=create_stamped_pose(FISH_BIN_FRAME),
        overwrite=True
    )

    set_pose_shark = py_trees.behaviours.SetBlackboardVariable(
        name="set bin target pose",
        variable_name=pose_key,
        variable_value=create_stamped_pose(SHARK_BIN_FRAME),
        overwrite=True
    )

    fish_setup_sequence.add_children(
        [
            check_is_fish,
            set_pose_fish
        ]
    )

    shark_setup_sequence.add_child(set_pose_shark)

    bin_selector_root.add_children(
        [
            fish_setup_sequence,
            shark_setup_sequence
        ]
    )
