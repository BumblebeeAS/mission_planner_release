import operator

import py_trees

from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto

########################## UPDATE CONSTANTS HERE #########################
BASE_LINK_FRAME = "auv4/base_link_ned"

# No Missing Transforms
SLALOM_ZERO_FRAME_CLUSTERED = "slalom_layer_0/clustered"
SLALOM_ONE_FRAME_CLUSTERED = "slalom_layer_1/clustered"
SLALOM_TWO_FRAME_CLUSTERED = "slalom_layer_2/clustered"

# One Missing Transform
# SLALOM_ZERO_FRAME_CLUSTERED
SLALOM_TWO_FROM_ONE_HARDCODED = "slalom_layer_2/hardcoded"

# Two Missing Transforms
SLALOM_ONE_FROM_ZERO_HARDCODED = "slalom_layer_1/hardcoded"
SLALOM_TWO_FROM_ONE_HARDCODED_HARDCODED = "slalom_layer_2/hardcoded/hardcoded"

WAIT_BETWEEN_MOVES = 30.0

# Gate Constants
#########################################################################

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

_IS_LEFT_KEY = "is_left_side"  # Global key for left option or not


def create_slalom_left_pose(frame_id: str):
    """
    Create a PoseStamped for the left side of the slalom.
    """
    return create_stamped_pose(
        frame_id=frame_id,
        position_x=0.75,
        position_y=0.5,
        position_z=0.0,
        roll=-90.0,
        pitch=-90.0,
        yaw=0.0,  # Facing left
    )


def create_slalom_right_pose(frame_id: str):
    """
    Create a PoseStamped for the right side of the slalom.
    """
    return create_stamped_pose(
        frame_id=frame_id,
        position_x=2.25,
        position_y=0.5,
        position_z=0.0,
        roll=-90.0,
        pitch=-90.0,
        yaw=0.0,
    )


def create_channel_movement_root(number_of_missing_channels: int):
    """
    Generate the root of the channel movement tree based on the number of missing channels.
    Will instantiate multiple of such trees, execution will be based on runtime check.
    This is because trees cannot be dynamically generated at runtime (i think).
    """
    root = py_trees.composites.Sequence(
        name="Channel Movement Task",
        memory=True,
    )

    # Left Check
    check_is_fish_zero = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check if left side zero",
        check=py_trees.common.ComparisonExpression(
            variable=_IS_LEFT_KEY,
            value=True,
            operator=lambda x, y: operator.__eq__(x, y),
        ),
    )

    check_is_fish_one = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check if left side zero",
        check=py_trees.common.ComparisonExpression(
            variable=_IS_LEFT_KEY,
            value=True,
            operator=lambda x, y: operator.__eq__(x, y),
        ),
    )

    check_is_fish_two = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check if left side zero",
        check=py_trees.common.ComparisonExpression(
            variable=_IS_LEFT_KEY,
            value=True,
            operator=lambda x, y: operator.__eq__(x, y),
        ),
    )
    # No Missing Transforms
    zero_root = py_trees.composites.Selector(
        name="Zero Missing Transforms",
        memory=True,
    )
    zero_left_seq = py_trees.composites.Sequence(
        name="Zero Left Side Sequence",
        memory=True,
        children=[
            check_is_fish_zero,
            goto.FromConstant(
                name="Goto zero left side zero",
                pose=create_slalom_left_pose(SLALOM_ZERO_FRAME_CLUSTERED),
            ),
            py_trees.timers.Timer(name="timer", duration=WAIT_BETWEEN_MOVES),
            goto.FromConstant(
                name="Goto zero left side one",
                pose=create_slalom_left_pose(SLALOM_ONE_FRAME_CLUSTERED),
            ),
            py_trees.timers.Timer(name="timer", duration=WAIT_BETWEEN_MOVES),
            goto.FromConstant(
                name="Goto zero left side two",
                pose=create_slalom_left_pose(SLALOM_TWO_FRAME_CLUSTERED),
            ),
        ],
    )

    zero_right_seq = py_trees.composites.Sequence(
        name="Zero Right Side Sequence",
        memory=True,
        children=[
            goto.FromConstant(
                name="Goto zero right side zero",
                pose=create_slalom_right_pose(SLALOM_ZERO_FRAME_CLUSTERED),
            ),
            py_trees.timers.Timer(name="timer", duration=WAIT_BETWEEN_MOVES),
            goto.FromConstant(
                name="Goto zero right side one",
                pose=create_slalom_right_pose(SLALOM_ONE_FRAME_CLUSTERED),
            ),
            py_trees.timers.Timer(name="timer", duration=WAIT_BETWEEN_MOVES),
            goto.FromConstant(
                name="Goto zero right side two",
                pose=create_slalom_right_pose(SLALOM_TWO_FRAME_CLUSTERED),
            ),
        ],
    )

    zero_root.add_children(
        children=[
            zero_left_seq,
            zero_right_seq,
        ]
    )

    # One Missing Transform
    one_root = py_trees.composites.Selector(
        name="One Missing Transform",
        memory=True,
    )

    one_left_seq = py_trees.composites.Sequence(
        name="One Left Side Sequence",
        memory=True,
        children=[
            check_is_fish_one,
            goto.FromConstant(
                name="Goto one left side zero",
                pose=create_slalom_left_pose(SLALOM_ZERO_FRAME_CLUSTERED),
            ),
            py_trees.timers.Timer(name="timer", duration=WAIT_BETWEEN_MOVES),
            goto.FromConstant(
                name="Goto one left side one",
                pose=create_slalom_left_pose(SLALOM_ONE_FRAME_CLUSTERED),
            ),
            py_trees.timers.Timer(name="timer", duration=WAIT_BETWEEN_MOVES),
            goto.FromConstant(
                name="Goto one left side two hardcoded",
                pose=create_slalom_left_pose(SLALOM_TWO_FROM_ONE_HARDCODED),
            ),
        ],
    )

    one_right_seq = py_trees.composites.Sequence(
        name="One Right Side Sequence",
        memory=True,
        children=[
            goto.FromConstant(
                name="Goto one right side zero",
                pose=create_slalom_right_pose(SLALOM_ZERO_FRAME_CLUSTERED),
            ),
            py_trees.timers.Timer(name="timer", duration=WAIT_BETWEEN_MOVES),
            goto.FromConstant(
                name="Goto one right side one",
                pose=create_slalom_right_pose(SLALOM_ONE_FRAME_CLUSTERED),
            ),
            py_trees.timers.Timer(name="timer", duration=WAIT_BETWEEN_MOVES),
            goto.FromConstant(
                name="Goto one right side two hardcoded",
                pose=create_slalom_right_pose(SLALOM_TWO_FROM_ONE_HARDCODED),
            ),
        ],
    )

    one_root.add_children(
        children=[
            one_left_seq,
            one_right_seq,
        ]
    )

    # Two Missing Transforms
    two_root = py_trees.composites.Selector(
        name="Two Missing Transforms",
        memory=True,
    )

    two_left_seq = py_trees.composites.Sequence(
        name="Two Left Side Sequence",
        memory=True,
        children=[
            check_is_fish_two,
            goto.FromConstant(
                name="Goto two left side zero",
                pose=create_slalom_left_pose(SLALOM_ZERO_FRAME_CLUSTERED),
            ),
            py_trees.timers.Timer(name="timer", duration=WAIT_BETWEEN_MOVES),
            goto.FromConstant(
                name="Goto two left side one hardcoded",
                pose=create_slalom_left_pose(SLALOM_ONE_FROM_ZERO_HARDCODED),
            ),
            py_trees.timers.Timer(name="timer", duration=WAIT_BETWEEN_MOVES),
            goto.FromConstant(
                name="Goto two left side two hardcoded hardcoded",
                pose=create_slalom_left_pose(SLALOM_TWO_FROM_ONE_HARDCODED_HARDCODED),
            ),
        ],
    )

    two_right_seq = py_trees.composites.Sequence(
        name="Two Right Side Sequence",
        memory=True,
        children=[
            goto.FromConstant(
                name="Goto two right side zero",
                pose=create_slalom_right_pose(SLALOM_ZERO_FRAME_CLUSTERED),
            ),
            py_trees.timers.Timer(name="timer", duration=WAIT_BETWEEN_MOVES),
            goto.FromConstant(
                name="Goto two right side one hardcoded",
                pose=create_slalom_right_pose(SLALOM_ONE_FROM_ZERO_HARDCODED),
            ),
            py_trees.timers.Timer(name="timer", duration=WAIT_BETWEEN_MOVES),
            goto.FromConstant(
                name="Goto two right side two hardcoded hardcoded",
                pose=create_slalom_right_pose(SLALOM_TWO_FROM_ONE_HARDCODED_HARDCODED),
            ),
        ],
    )

    two_root.add_children(
        children=[
            two_left_seq,
            two_right_seq,
        ]
    )

    # Add the roots to the main root based on the number of missing channels
    if number_of_missing_channels == 0:
        root.add_child(zero_root)
    elif number_of_missing_channels == 1:
        root.add_child(one_root)
    elif number_of_missing_channels == 2:
        root.add_child(two_root)
    else:
        raise ValueError("Invalid number of missing channels. Must be 0, 1, or 2.")

    # Currently, the tree does not support the case where all three channels are missing.
    return root
