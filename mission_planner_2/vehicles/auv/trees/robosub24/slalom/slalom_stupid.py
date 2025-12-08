import operator
import os
from typing import Literal

import py_trees
import py_trees_ros
import yaml
from ament_index_python.packages import get_package_share_directory

from mission_planner_2.common.util.pose_utils import create_stamped_pose
from mission_planner_2.vehicles.auv.trees.robosub24.goto import goto

########################## UPDATE CONSTANTS HERE #########################
VISION_SERVER_TOPIC = "/auv4/slalom/manage_nodes"
DEPTH_ANYTHING_SERVER_TOPIC = "/auv4/slalom/manage_components"

BASE_LINK_FRAME = "auv4/base_link_ned"
WORLD_FRAME = "world_ned"
DEPTH_OVERRIDE_VALUE = 0.9

SLALOM_LAYER_ZERO = "map_ned/slalom/stupid_0"
SLALOM_LAYER_ONE = "map_ned/slalom/stupid_1"
SLALOM_LAYER_TWO = "map_ned/slalom/stupid_2"

# set by  gate task if there change must change here too
IS_LEFT_KEY = "/global/is_left_side"  # Global key for left option or not
RIGHT_OFFSET = 0.6
LEFT_OFFSET = -0.6
#########################################################################


def create_move_slalom_centre_root():
    root = py_trees.composites.Selector(name="Move to centre post", memory=True)

    seq_move_centre_left = py_trees.composites.Sequence(
        name="Move post left",
        memory=True,
    )

    check_is_left = py_trees.behaviours.CheckBlackboardVariableValue(
        name="check is left",
        check=py_trees.common.ComparisonExpression(
            variable=IS_LEFT_KEY,
            value=True,
            operator=operator.eq,
        ),
    )

    goto_left = goto.FromConstant(
        name="Move to post left",
        pose=create_stamped_pose(
            frame_id=BASE_LINK_FRAME,
            position_x=2.0,
            position_y=0.75,
        ),
        depth_override_value=DEPTH_OVERRIDE_VALUE,
    )

    seq_move_centre_left.add_children(
        [
            check_is_left,
            goto_left,
        ]
    )

    goto_right = goto.FromConstant(
        name="Move to post right",
        pose=create_stamped_pose(
            frame_id=BASE_LINK_FRAME,
            position_x=2.0,
            position_y=-0.75,
        ),
        depth_override_value=DEPTH_OVERRIDE_VALUE,
    )

    root.add_children(
        [
            seq_move_centre_left,
            goto_right,
        ]
    )

    return root


def create_move_to_layer_root(coords: dict):
    root = py_trees.composites.Selector(
        name=f"Movement to stupid",
        memory=True,
    )

    seq_move_left_stupid = py_trees.composites.Sequence(
        name=f"Move left of stupid",
        memory=True,
    )

    check_is_left = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check is left",
        check=py_trees.common.ComparisonExpression(
            variable=IS_LEFT_KEY,
            value=True,
            operator=operator.eq,
        ),
    )

    goto_left_0 = goto.FromConstant(
        name=f"Move to left for layer 0",
        pose=create_stamped_pose(
            frame_id=BASE_LINK_FRAME,
            position_x=coords["slalom_stupid_0"]["x"],
            position_y=coords["slalom_stupid_0"]["y"] + LEFT_OFFSET,
        ),
        depth_override_value=DEPTH_OVERRIDE_VALUE,
    )
    goto_left_1 = goto.FromConstant(
        name=f"Move to left for layer 1",
        pose=create_stamped_pose(
            frame_id=BASE_LINK_FRAME,
            position_x=coords["slalom_stupid_1"]["x"],
            position_y=coords["slalom_stupid_1"]["y"],
        ),
        depth_override_value=DEPTH_OVERRIDE_VALUE,
    )
    goto_left_2 = goto.FromConstant(
        name=f"Move to left for layer 2",
        pose=create_stamped_pose(
            frame_id=BASE_LINK_FRAME,
            position_x=coords["slalom_stupid_2"]["x"],
            position_y=coords["slalom_stupid_2"]["y"],
        ),
        depth_override_value=DEPTH_OVERRIDE_VALUE,
    )

    seq_move_left_stupid.add_children(
        [
            check_is_left,
            goto_left_0,
            goto_left_1,
            goto_left_2,
        ]
    )

    seq_move_right_stupid = py_trees.composites.Sequence(
        name=f"Move right to stupid",
        memory=True,
    )
    goto_right_0 = goto.FromConstant(
        name=f"Move to right for layer 0",
        pose=create_stamped_pose(
            frame_id=BASE_LINK_FRAME,
            position_x=coords["slalom_stupid_0"]["x"],
            position_y=coords["slalom_stupid_0"]["y"] + RIGHT_OFFSET,
        ),
        depth_override_value=DEPTH_OVERRIDE_VALUE,
    )
    goto_right_1 = goto.FromConstant(
        name=f"Move to right for layer 1",
        pose=create_stamped_pose(
            frame_id=BASE_LINK_FRAME,
            position_x=coords["slalom_stupid_1"]["x"],
            position_y=coords["slalom_stupid_1"]["y"],
        ),
        depth_override_value=DEPTH_OVERRIDE_VALUE,
    )
    goto_right_2 = goto.FromConstant(
        name=f"Move to right for layer 2",
        pose=create_stamped_pose(
            frame_id=BASE_LINK_FRAME,
            position_x=coords["slalom_stupid_2"]["x"],
            position_y=coords["slalom_stupid_2"]["y"],
        ),
        depth_override_value=DEPTH_OVERRIDE_VALUE,
    )

    seq_move_right_stupid.add_children(
        [
            goto_right_0,
            goto_right_1,
            goto_right_2,
        ]
    )

    root.add_children(
        [
            seq_move_left_stupid,
            seq_move_right_stupid,
        ]
    )

    return root


def create_slalom_stupid_root(coords: dict):
    root = py_trees.composites.Sequence(
        name=f"Move to slalom stupid",
        memory=True,
    )
    move_to_stupids = create_move_to_layer_root(coords)
    move_to_post = create_move_slalom_centre_root()

    root.add_children(
        [
            move_to_stupids,
            move_to_post,
        ]
    )

    return root
