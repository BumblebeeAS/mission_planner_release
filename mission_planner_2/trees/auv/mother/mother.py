import os

import py_trees
import yaml
from ament_index_python.packages import get_package_share_directory

from mission_planner_2.trees.auv.bins.bins import create_bin_root
from mission_planner_2.trees.auv.button.wait_for_button import (
    create_wait_for_button_root,
)
from mission_planner_2.trees.auv.gate.gate import create_gate_root
from mission_planner_2.trees.auv.gate.move_to_task import create_move_to_gate_task_root
from mission_planner_2.trees.auv.gate.return_home import create_return_root
from mission_planner_2.trees.auv.mother.move_to_task import create_move_to_task
from mission_planner_2.trees.auv.octagon.octagon import create_octagon_root
from mission_planner_2.trees.auv.slalom.slalom import create_slalom_root
from mission_planner_2.trees.auv.torpedo.torpedo import create_torpedo_root

BUTTON_TOPIC = "/auv4/button/left"
IS_LEFT_KEY = "/global/is_left_side"  # Global key for left option or not
BASE_LINK_KEY = "/global/base_link"
WORLD_KEY = "/global/world"
CURRENT_ODOM_KEY = "/global/current_odom"
ZERO_YAW_POSE_KEY = "/global/zero_yaw_pose_key"


def load_mission_coordinates():
    package_share_directory = get_package_share_directory("mission_planner_2")
    yaml_file_path = os.path.join(package_share_directory, "cfg", "static_tfs.yaml")

    with open(yaml_file_path, "r") as file:
        data = yaml.safe_load(file)

    coords = {}
    for item in data["map"]:
        name = item["name"]
        coords[name] = {
            "x": item.get("x", 0.0),
            "y": item.get("y", 0.0),
            "z": item.get("z", 0.0),
            "roll": item.get("roll", 0.0),
            "pitch": item.get("pitch", 0.0),
            "yaw": item.get("yaw", 0.0),
        }

    return coords


def create_mother(coords: dict):
    root = py_trees.composites.Sequence(
        name="mother",
        memory=True,
    )

    wait_for_button = create_wait_for_button_root(
        button_topic=BUTTON_TOPIC,
        num_retries=1000000,
    )

    set_base_link_frame = py_trees.behaviours.SetBlackboardVariable(
        name="Set Base Link Frame",
        variable_name=BASE_LINK_KEY,
        variable_value="auv4/base_link_ned",
        overwrite=True,
    )

    set_world_frame = py_trees.behaviours.SetBlackboardVariable(
        name="Set World Frame",
        variable_name=WORLD_KEY,
        variable_value="world_ned",
        overwrite=True,
    )

    gate_root = create_gate_root()
    move_to_gate = create_move_to_gate_task_root(
        world_coords=coords["gate_start"],
        relative_coords=coords["rel_gate_start"],
        flipped_relative_coords=coords["rel_gate_start_flip"],
        is_relative=False,
        is_flip=True,
    )

    move_to_slalom = create_move_to_task(
        task="slalom",
        start=coords["gate_end"],
        end=coords["slalom_start"],
        odom_key=CURRENT_ODOM_KEY,
        zero_yaw_pose_key=ZERO_YAW_POSE_KEY,
    )
    slalom_root = create_slalom_root()

    move_to_bin = create_move_to_task(
        task="bin",
        start=coords["slalom_end"],
        end=coords["bin"],
        odom_key=CURRENT_ODOM_KEY,
        zero_yaw_pose_key=ZERO_YAW_POSE_KEY,
    )
    bin_root = create_bin_root()

    move_to_torpedo = create_move_to_task(
        task="torpedo",
        start=coords["bin"],
        end=coords["torpedo_start"],
        odom_key=CURRENT_ODOM_KEY,
        zero_yaw_pose_key=ZERO_YAW_POSE_KEY,
    )
    torpedo_root = create_torpedo_root()

    move_to_space = create_move_to_task(
        task="post_torpedo",
        start=coords["torpedo_start"],
        end=coords["torpedo_post"],
        odom_key=CURRENT_ODOM_KEY,
        zero_yaw_pose_key=ZERO_YAW_POSE_KEY,
    )
    move_to_octagon = create_move_to_task(
        task="octagon",
        start=coords["torpedo_post"],
        end=coords["octagon"],
        odom_key=CURRENT_ODOM_KEY,
        zero_yaw_pose_key=ZERO_YAW_POSE_KEY,
    )
    octagon_root = create_octagon_root()

    # TODO: see if need a move to gate here to go closer to do the return task
    return_root = create_return_root()

    # TODO: PURELY FOR TESTING
    set_is_left = py_trees.behaviours.SetBlackboardVariable(
        name="Set is left for test",
        variable_name=IS_LEFT_KEY,
        variable_value=True,
        overwrite=True,
    )

    root.add_children(
        [
            # wait_for_button,
            # set_is_left,
            set_base_link_frame,
            set_world_frame,  # TODO: use multi set bb?
            move_to_gate,
            # gate_root,
            # move_to_slalom,
            # slalom_root,
            # move_to_bin,
            # bin_root,
            # move_to_torpedo,
            # torpedo_root,
            # move_to_space,
            # move_to_octagon,
            # octagon_root,
            # return_root,
        ]
    )

    return root
