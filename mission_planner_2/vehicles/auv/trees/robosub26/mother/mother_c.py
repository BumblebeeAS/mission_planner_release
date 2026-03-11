import os

import py_trees
import py_trees_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger

from mission_planner_2.vehicles.auv.trees.goto import goto
from mission_planner_2.common.util.pose_utils import (
    create_stamped_pose
)

######################### TASK ROOTS #########################

from mission_planner_2.vehicles.auv.trees.robosub26.gate.gate import create_gate_root

from mission_planner_2.vehicles.auv.trees.robosub26.gate.return_home import create_return_root

##############################################################



######################### CONSTANTS #########################

BASE_LINK_FRAME = "auv4/base_link_ned"

#############################################################





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

    seq_reset_clustering.add_children(
        [
            srv_reset_cluster_tf_action,
            # srv_reset_cluster_tf_multi_action, # left here as reminder that others cluster resets exist
        ]
    )

    gate_root = create_gate_root()
    force_succeed_gate = py_trees.decorators.FailureIsSuccess(
        name="Force succeed gate",
        child=gate_root,
    )

    goto_yaw_back = goto.FromConstant(
        name="turn around to face gate again",
        pose=create_stamped_pose(BASE_LINK_FRAME, yaw=180.0),
    )

    return_home_root = create_return_root()
    force_succeed_return = py_trees.decorators.FailureIsSuccess(
        name="Force succeed return home",
            child=return_home_root
    )

    root.add_children([
        seq_reset_clustering,
        force_succeed_gate,
        goto_yaw_back,
        force_succeed_return,
    ])

    return root
