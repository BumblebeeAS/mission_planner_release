import operator

import py_trees
import py_trees_ros
from rclpy.qos import qos_profile_system_default

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

WAIT_BETWEEN_MOVES = 10.0
TRANSFORM_CHECK_TIMEOUT = 5.0

# Gate Constants
#########################################################################

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

_IS_LEFT_KEY = "/global/is_left_side"  # Global key for left option or not


def create_slalom_left_pose(frame_id: str):
    """
    Create a PoseStamped for the left side of the slalom.
    """
    return create_stamped_pose(
        frame_id=frame_id,
        position_x=0.75,
        position_y=0.3,
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
        position_y=0.3,
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

    check_is_fish_one_dup = py_trees.behaviours.CheckBlackboardVariableValue(
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
                pose=[
                    create_slalom_left_pose(SLALOM_ZERO_FRAME_CLUSTERED),
                    create_slalom_left_pose(SLALOM_ONE_FRAME_CLUSTERED),
                    create_slalom_left_pose(SLALOM_TWO_FRAME_CLUSTERED),
                ],
                specified_heading=False,
            ),
        ],
    )

    zero_right_seq = py_trees.composites.Sequence(
        name="Zero Right Side Sequence",
        memory=True,
        children=[
            goto.FromConstant(
                name="Goto zero right side zero",
                pose=[
                    create_slalom_right_pose(SLALOM_ZERO_FRAME_CLUSTERED),
                    create_slalom_right_pose(SLALOM_ONE_FRAME_CLUSTERED),
                    create_slalom_right_pose(SLALOM_TWO_FRAME_CLUSTERED),
                ],
                specified_heading=False,
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

    one_missing_layer_one = py_trees.composites.Sequence(
        name="One Missing Transform with Layer One Missing",
        memory=True,
    )

    one_missing_layer_one_check = py_trees_ros.transforms.ToBlackboard(
        name="One Missing Transform with Layer One Missing Check",
        variable_name=fk("one_missing_layer_one"),
        target_frame=SLALOM_ONE_FRAME_CLUSTERED,
        source_frame=BASE_LINK_FRAME,
        qos_profile=qos_profile_system_default,
    )

    one_missing_layer_one_sel = py_trees.composites.Selector(
        name="One Missing Transform with Layer One Missing Selector",
        memory=True,
    )

    one_missing_layer_one_left_seq = py_trees.composites.Sequence(
        name="One Missing Transform with Layer One Missing Left Sequence",
        memory=True,
        children=[
            check_is_fish_one,
            goto.FromConstant(
                name="Goto one layer one missing left side zero",
                pose=[
                    create_slalom_left_pose(SLALOM_ZERO_FRAME_CLUSTERED),
                    create_slalom_left_pose(SLALOM_ONE_FROM_ZERO_HARDCODED),
                    create_slalom_left_pose(SLALOM_TWO_FRAME_CLUSTERED),
                ],
                specified_heading=False,
            ),
        ],
    )

    one_missing_layer_one_right_seq = py_trees.composites.Sequence(
        name="One Missing Transform with Layer One Missing Right Sequence",
        memory=True,
        children=[
            goto.FromConstant(
                name="Goto one layer one missing right side zero",
                pose=[
                    create_slalom_right_pose(SLALOM_ZERO_FRAME_CLUSTERED),
                    create_slalom_right_pose(SLALOM_ONE_FROM_ZERO_HARDCODED),
                    create_slalom_right_pose(SLALOM_TWO_FRAME_CLUSTERED),
                ],
                specified_heading=False,
            ),
        ],
    )

    one_missing_layer_one_sel.add_children(
        children=[
            one_missing_layer_one_left_seq,
            one_missing_layer_one_right_seq,
        ]
    )

    one_missing_layer_one.add_children(
        children=[
            py_trees.decorators.Timeout(
                name="One Missing Transform Layer One Check Timeout",
                child=one_missing_layer_one_check,
                duration=TRANSFORM_CHECK_TIMEOUT,
            ),
            one_missing_layer_one_sel,
        ]
    )

    one_missing_layer_two = py_trees.composites.Selector(
        name="One Missing Transform with Layer Two Missing",
        memory=True,
    )

    one_missing_layer_two_left_seq = py_trees.composites.Sequence(
        name="One Missing Transform with Layer Two Missing Left Sequence",
        memory=True,
        children=[
            check_is_fish_one_dup,
            goto.FromConstant(
                name="Goto one layer two missing left side zero",
                pose=[
                    create_slalom_left_pose(SLALOM_ZERO_FRAME_CLUSTERED),
                    create_slalom_left_pose(SLALOM_ONE_FRAME_CLUSTERED),
                    create_slalom_left_pose(SLALOM_TWO_FROM_ONE_HARDCODED),
                ],
                specified_heading=False,
            ),
        ],
    )

    one_missing_layer_two_right_seq = py_trees.composites.Sequence(
        name="One Missing Transform with Layer Two Missing Right Sequence",
        memory=True,
        children=[
            goto.FromConstant(
                name="Goto one layer two missing right side zero",
                pose=[
                    create_slalom_right_pose(SLALOM_ZERO_FRAME_CLUSTERED),
                    create_slalom_right_pose(SLALOM_ONE_FRAME_CLUSTERED),
                    create_slalom_right_pose(SLALOM_TWO_FROM_ONE_HARDCODED),
                ],
                specified_heading=False,
            ),
        ],
    )

    one_missing_layer_two.add_children(
        children=[
            one_missing_layer_two_left_seq,
            one_missing_layer_two_right_seq,
        ]
    )

    one_root.add_children(
        children=[
            one_missing_layer_one,
            one_missing_layer_two,
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
                pose=[
                    create_slalom_left_pose(SLALOM_ZERO_FRAME_CLUSTERED),
                    create_slalom_left_pose(SLALOM_ONE_FROM_ZERO_HARDCODED),
                    create_slalom_left_pose(SLALOM_TWO_FROM_ONE_HARDCODED_HARDCODED),
                ],
                specified_heading=False,
            ),
        ],
    )

    two_right_seq = py_trees.composites.Sequence(
        name="Two Right Side Sequence",
        memory=True,
        children=[
            goto.FromConstant(
                name="Goto two right side zero",
                pose=[
                    create_slalom_right_pose(SLALOM_ZERO_FRAME_CLUSTERED),
                    create_slalom_right_pose(SLALOM_ONE_FROM_ZERO_HARDCODED),
                    create_slalom_right_pose(SLALOM_TWO_FROM_ONE_HARDCODED_HARDCODED),
                ],
                specified_heading=False,
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
