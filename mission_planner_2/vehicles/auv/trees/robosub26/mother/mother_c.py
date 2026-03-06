import os

import py_trees
import py_trees_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger

from mission_planner_2.vehicles.auv.trees.robosub24.gate.gate import create_gate_root

# from mission_planner_2.vehicles.shared.trees.blackboard import MultiSetBlackboard

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

    gate_root = create_gate_root()
    force_succeed_gate = py_trees.decorators.FailureIsSuccess(
        name="Force succeed gate",
        child=gate_root,
    )


    root.add_children(
        [
            force_succeed_gate
        ]
    )

    return root
