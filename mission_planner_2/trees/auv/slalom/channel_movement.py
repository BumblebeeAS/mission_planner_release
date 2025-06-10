import py_trees

from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto

########################## UPDATE CONSTANTS HERE #########################
BASE_LINK_FRAME = "auv4/base_link_ned"
CHANNEL_PAIR_ONE_FRAME = "slalom_layer_0"
CHANNEL_PAIR_TWO_FRAME = "slalom_layer_1"
CHANNEL_PAIR_THREE_FRAME = "slalom_layer_2"

CHANNEL_PAIR_ONE_FRAME_CLUSTERED = "slalom_layer_0/clustered"
CHANNEL_PAIR_TWO_FRAME_CLUSTERED = "slalom_layer_1/clustered"
CHANNEL_PAIR_THREE_FRAME_CLUSTERED = "slalom_layer_2/clustered"

# TODO: Update the following in cfg.yaml
# Hardcoded transform defined from channel two to three, two is clustered/hardcoded
CHANNEL_PAIR_THREE_FROM_TWO_HARDCODE = "slalom_layer_2/hardcoded"

# Hardcoded transform defined from channel one to two, one is clustered
CHANNEL_PAIR_TWO_FROM_ONE_HARDCODE = "slalom_layer_1/hardcoded"
#########################################################################


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

    move_channel_one = goto.FromConstant(
        name="Goto channel one",
        pose=create_stamped_pose(CHANNEL_PAIR_ONE_FRAME_CLUSTERED),
    )

    move_channel_two = goto.FromConstant(
        name="Goto channel two",
        pose=create_stamped_pose(CHANNEL_PAIR_TWO_FRAME_CLUSTERED),
    )

    move_channel_three = goto.FromConstant(
        name="Goto channel three",
        pose=create_stamped_pose(CHANNEL_PAIR_THREE_FRAME_CLUSTERED),
    )

    move_channel_three_hardcode = goto.FromConstant(
        name="Goto channel three hardcode",
        pose=create_stamped_pose(CHANNEL_PAIR_THREE_FROM_TWO_HARDCODE),
    )

    move_channel_two_hardcode = goto.FromConstant(
        name="Goto channel two hardcode",
        pose=create_stamped_pose(CHANNEL_PAIR_TWO_FROM_ONE_HARDCODE),
    )

    if number_of_missing_channels == 0:
        root.add_children(
            children=[move_channel_one, move_channel_two, move_channel_three]
        )
    elif number_of_missing_channels == 1:
        root.add_children(
            children=[move_channel_one, move_channel_two, move_channel_three_hardcode]
        )
    elif number_of_missing_channels == 2:
        root.add_children(
            children=[
                move_channel_one,
                move_channel_two_hardcode,
                move_channel_three_hardcode,
            ]
        )

    # Currently, the tree does not support the case where all three channels are missing.
    return root
