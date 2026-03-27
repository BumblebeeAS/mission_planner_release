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





def create_mother():
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
    goto_dive = goto.FromConstant(
        name="Dive",
        pose=create_stamped_pose(BASE_LINK_FRAME, position_x=0.0, position_y=0.0, position_z=0.8 ),
        stabilize_duration=30,
        z_threshold=0.02
    )
    go_left = goto.FromConstant(
        name="Go left",
        pose=create_stamped_pose(BASE_LINK_FRAME, position_x=0.0, position_y=-1.0),
        stabilize_duration=30
    )
    go_right = goto.FromConstant(
        name="Go right",
        pose=create_stamped_pose(BASE_LINK_FRAME, position_x=0.0, position_y=1.0),
        stabilize_duration=30
    )
    goto_u_turn_right = goto.FromConstant(
        name="forward",
        pose=create_stamped_pose(BASE_LINK_FRAME, position_x=10.0, position_y=1.0),
        stabilize_duration=30,
        yaw_threshold=0.1
    )
    goto_u_turn = goto.FromConstant(
        name="Do u-turn",
        pose=create_stamped_pose(BASE_LINK_FRAME, position_y=-2.0, yaw=180.0),
        yaw_threshold=0.1,
        stabilize_duration=30,

    )

    goto_approach_gate = goto.FromConstant(
        name="forward again",
        pose=create_stamped_pose(BASE_LINK_FRAME, position_x=9.5, position_y=1.0),
        yaw_threshold=0.1,
        stabilize_duration=30,

    )

    goto_turnaround = goto.FromConstant(
        name="turnaround",
        pose=create_stamped_pose(BASE_LINK_FRAME, yaw=180.0),
        yaw_threshold=0.1,
        stabilize_duration=30,

    )

    return_home_root = create_return_root()
    force_succeed_return = py_trees.decorators.FailureIsSuccess(
        name="Force succeed return home",
            child=return_home_root
    )

    # children = [goto_dive]
    # for _ in range(10):
    #     children.append(goto.FromConstant(name="Go left", pose=create_stamped_pose(BASE_LINK_FRAME, position_x=0.0, position_y=-1.0), stabilize_duration=3))
    #     children.append(goto.FromConstant(name="Go right", pose=create_stamped_pose(BASE_LINK_FRAME, position_x=0.0, position_y=1.0), stabilize_duration=3))

    # root.add_children(children)

    root.add_children([
        seq_reset_clustering,
        force_succeed_gate,
        goto_u_turn_right,
        goto_u_turn,
        goto_approach_gate,
        # goto_turnaround,
        force_succeed_return,
    ])      

    return root
