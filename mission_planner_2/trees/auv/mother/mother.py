import os

import py_trees
import py_trees_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger

from mission_planner_2.trees.auv.acoustics.acoustics import create_acoustics_root
from mission_planner_2.trees.auv.bins.bins import create_bin_root
from mission_planner_2.trees.auv.gate.gate import create_gate_root
from mission_planner_2.trees.auv.gate.move_to_task import create_move_to_gate_task_root
from mission_planner_2.trees.auv.mother.button_behaviors import create_button_start_root
from mission_planner_2.trees.auv.mother.move_to_task import create_move_to_task
from mission_planner_2.trees.auv.slalom.slalom import create_slalom_root

LEFT_BUTTON_TOPIC = "/auv4/button/left"
RIGHT_BUTTON_TOPIC = "/auv4/button/right"
IS_LEFT_KEY = "/global/is_left_side"  # Global key for left option or not
BASE_LINK_KEY = "/global/base_link"
WORLD_KEY = "/global/world"
CURRENT_ODOM_KEY = "/global/current_odom"
ZERO_YAW_POSE_KEY = "/global/zero_yaw_pose_key"
CHOICE_KEY = "/global/choice_is_fish"
CONTROLS_SRV_TOPIC = "/auv4/controls/controller"
RESET_POSE_SRV_TOPIC = "/auv4/nav/reset_pose"
YAW_BEFORE_GATE_KEY = "/global/yaw_before_gate"

BUTTON_RETRIES = 1000000


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

    button_start = create_button_start_root(
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

    gate_root = create_gate_root()

    move_to_gate = create_move_to_gate_task_root(
        world_coords=coords["gate_start"],
        relative_coords=coords["rel_gate_start"],
        flipped_relative_coords=coords["rel_gate_start_flip"],
        is_relative=True,
        is_flip=False,
    )

    move_to_slalom = create_move_to_task(
        task="slalom",
        start=coords["gate_end"],
        end=coords["slalom_start"],
        odom_key=CURRENT_ODOM_KEY,
        zero_yaw_pose_key=ZERO_YAW_POSE_KEY,
    )

    slalom_root = create_slalom_root()

    move_to_slalom_end = create_move_to_task(
        task="move_to_slalom_end",
        start=coords["slalom_start"],
        end=coords["slalom_end"],
        odom_key=CURRENT_ODOM_KEY,
        zero_yaw_pose_key=ZERO_YAW_POSE_KEY,
    )

    move_to_bin = create_move_to_task(
        task="bin",
        start=coords["slalom_end"],
        end=coords["bin"],
        odom_key=CURRENT_ODOM_KEY,
        zero_yaw_pose_key=ZERO_YAW_POSE_KEY,
        specified_heading=False,
    )

    bin_root = create_bin_root()

    move_to_acoustic_start = create_move_to_task(
        task="acoustic_start",
        start=coords["bin"],
        end=coords["acoustic_start"],
        odom_key=CURRENT_ODOM_KEY,
        zero_yaw_pose_key=ZERO_YAW_POSE_KEY,
    )

    def move_func(start_coords, end_coords):
        return create_move_to_task(
            task=f"Acoustic move from {start_coords} to {end_coords}",
            start=coords[start_coords],
            end=coords[end_coords],
            odom_key=CURRENT_ODOM_KEY,
            zero_yaw_pose_key=ZERO_YAW_POSE_KEY,
            specified_heading=False,
        )

    acoustics_root = create_acoustics_root(move_func)

    root.add_children(
        [
            button_start,
            seq_reset_clustering,
            srv_get_choice,
            set_base_link_frame,
            set_world_frame,  # TODO: use multi set bb?
            move_to_gate,
            gate_root,
            move_to_slalom,
            # move_to_slalom_end,
            slalom_root,
            move_to_bin,
            bin_root,
            move_to_acoustic_start,
            acoustics_root,
        ]
    )

    return root
