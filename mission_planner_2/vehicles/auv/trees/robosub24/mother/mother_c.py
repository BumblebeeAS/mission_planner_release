import os

import py_trees
import py_trees_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger

from mission_planner_2.vehicles.auv.trees.robosub24.acoustics.acoustics import (
    create_acoustics_root,
)
from mission_planner_2.vehicles.auv.trees.robosub24.bins.bins import create_bin_root
from mission_planner_2.vehicles.auv.trees.robosub24.gate.gate import create_gate_root
from mission_planner_2.vehicles.auv.trees.robosub24.mother.button_behaviors import (
    create_button_start_coinflip_root,
)
from mission_planner_2.vehicles.auv.trees.robosub24.mother.move_to_task import (
    create_move_to_task,
)
from mission_planner_2.vehicles.auv.trees.robosub24.octagon.octagon import (
    create_octagon_root,
)
from mission_planner_2.vehicles.auv.trees.robosub24.torpedo.torpedo import (
    create_torpedo_root,
)
from mission_planner_2.vehicles.shared.trees.blackboard import MultiSetBlackboard

LEFT_BUTTON_TOPIC = "/auv4/button/left"
RIGHT_BUTTON_TOPIC = "/auv4/button/right"
IS_LEFT_KEY = "/global/is_left_side"  # Global key for left option or not
BASE_LINK_KEY = "/global/base_link"
WORLD_KEY = "/global/world"
CURRENT_ODOM_KEY = "/global/current_odom"
ZERO_YAW_KEY = "/global/zero_yaw_key"
CHOICE_KEY = "/global/choice_is_fish"
CONTROLS_SRV_TOPIC = "/auv4/controls/controller"
RESET_POSE_SRV_TOPIC = "/auv4/nav/reset_pose"
YAW_BEFORE_GATE_KEY = "/global/yaw_before_gate"

BUTTON_RETRIES = 1000000
ACOUSTIC_TIMEOUT = 10.0
SLALOM_DEPTH = 0.15

IS_OCTAGON_ON_RIGHT = True


def load_mission_coordinates():
    package_share_directory = get_package_share_directory("mission_planner_2")
    yaml_file_path = os.path.join(package_share_directory, "cfg", "eyeball.yaml")

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

    button_coin_flip_start = create_button_start_coinflip_root(
        reset_pose_srv_topic=RESET_POSE_SRV_TOPIC,
        controls_srv_topic=CONTROLS_SRV_TOPIC,
        left_button_topic=LEFT_BUTTON_TOPIC,
        right_button_topic=RIGHT_BUTTON_TOPIC,
        button_retries=BUTTON_RETRIES,
    )

    seq_reset_clustering = py_trees.composites.Sequence(
        name="Reset clustering caches",
        memory=True,
    )

    srv_reset_cluster_tf_action = py_trees_ros.service_clients.FromConstant(
        name="Reset cluster tf action",
        service_type=Trigger,
        service_name="/auv4/cluster_tf/reset_caches",
        service_request=Trigger.Request(),
    )
    srv_reset_cluster_tf_multi_action = py_trees_ros.service_clients.FromConstant(
        name="Reset cluster tf multi action",
        service_type=Trigger,
        service_name="/auv4/cluster_tf_multi/reset_caches",
        service_request=Trigger.Request(),
    )
    srv_reset_cluster_tf_srv = py_trees_ros.service_clients.FromConstant(
        name="Reset cluster tf server",
        service_type=Trigger,
        service_name="/auv4/cluster_tfs_srv/reset_caches",
        service_request=Trigger.Request(),
    )
    srv_reset_cluster_tf_multi_srv = py_trees_ros.service_clients.FromConstant(
        name="Reset cluster tf multi server",
        service_type=Trigger,
        service_name="/auv4/cluster_tfs_multi_srv/reset_caches",
        service_request=Trigger.Request(),
    )

    seq_reset_clustering.add_children(
        [
            srv_reset_cluster_tf_action,
            srv_reset_cluster_tf_multi_action,
            srv_reset_cluster_tf_srv,
            srv_reset_cluster_tf_multi_srv,
        ]
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

    srv_get_choice = py_trees_ros.service_clients.FromConstant(
        name="Get Choice",
        service_name="/auv4/choice/get_is_fish",
        service_type=Trigger,
        service_request=Trigger.Request(),
        key_response=CHOICE_KEY,
    )

    move_to_gate = create_move_to_task(
        task="gate",
        start=coords["start"],
        end=coords["gate_start"],
        zero_yaw_key=ZERO_YAW_KEY,
    )

    gate_root = create_gate_root()
    force_succeed_gate = py_trees.decorators.FailureIsSuccess(
        name="Force succeed gate",
        child=gate_root,
    )

    bin_root = create_bin_root()
    force_succeed_bin = py_trees.decorators.FailureIsSuccess(
        name="Force succeed bin",
        child=bin_root,
    )

    move_to_acoustic_start = create_move_to_task(
        task="acoustic_start",
        start=coords["gate_end"],  # used to be bin
        end=coords["acoustic_start"],
        zero_yaw_key=ZERO_YAW_KEY,
    )

    def move_func(start_coords, end_coords, goto_depth, specified_heading):
        return create_move_to_task(
            task=f"Acoustic move from {start_coords} to {end_coords}",
            start=coords[start_coords],
            end=coords[end_coords],
            zero_yaw_key=ZERO_YAW_KEY,
            goto_depth=goto_depth,
            specified_heading=specified_heading,
        )

    def octagon_root():
        return create_octagon_root(
            world_to_table_yaw=coords["table"]["yaw"],
            zero_yaw_key=ZERO_YAW_KEY,
        )

    def torpedo_root():
        return create_torpedo_root(
            world_to_torp_yaw=coords["torpedo_with_yaw"]["yaw"],
            zero_yaw_key=ZERO_YAW_KEY,
        )

    acoustics_root = create_acoustics_root(
        move_func,
        octagon_root=octagon_root,
        torpedo_root=torpedo_root,
        timeout=ACOUSTIC_TIMEOUT,
        is_octagon_on_right=IS_OCTAGON_ON_RIGHT,
    )

    set_keys = MultiSetBlackboard(
        name="Set is_left, zero_yaw",
        keys=[IS_LEFT_KEY, ZERO_YAW_KEY],
        values=[True, 0.0],
        overwrite=True,
    )

    root.add_children(
        [
            button_coin_flip_start,
            seq_reset_clustering,
            srv_get_choice,
            set_base_link_frame,
            set_world_frame,  # TODO: use multi set bb?
            set_keys,  # if dont do gate
            move_to_gate,
            force_succeed_gate,
            move_to_acoustic_start,
            acoustics_root,  # move to bin is done inside acoustics root
            force_succeed_bin,
        ]
    )

    return root
