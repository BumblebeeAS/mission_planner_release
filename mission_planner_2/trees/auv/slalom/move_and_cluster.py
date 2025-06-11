import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from geometry_msgs.msg import PoseStamped

from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_slalom_clustering_goal
from mission_planner_2.trees.auv.goto import goto

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

"""
General Idea:
Move to defined location
Stabilise
Send clustering goal
"""

######################### UPDATE CONSTANTS HERE #########################
CLUSTERING_DURATION = 40
STABILIZE_CONTROLS_DURATION = 10
BASE_LINK_FRAME = "auv4/base_link_ned"
#########################################################################


def create_move_and_cluster_root(
    pose_stamped: PoseStamped,
):
    """
    Create the root of the move and cluster tree.

    Move to a specified location, wait for the controls to stabilize,
    and then perform clustering.

    Args:
        pose_stamped: PoseStamped object defining the target location to move to.
    """
    seq_root = py_trees.composites.Sequence(
        name="Move and cluster",
        memory=True,
    )

    goto_view_location = goto.FromConstant(name="Goto view location", pose=pose_stamped)

    timer_wait_stabilize = py_trees.timers.Timer(
        name="Stabilize before clustering",
        duration=STABILIZE_CONTROLS_DURATION,
    )

    # Multiple clustering goal
    action_clusters = py_trees_ros.action_clients.FromConstant(
        name="Cluster transforms",
        action_type=ClusterTf,
        action_name="/auv4/slalom",
        action_goal=create_slalom_clustering_goal(
            duration=CLUSTERING_DURATION,
        ),
    )

    # Add the children to the root sequence
    seq_root.add_children(
        [
            goto_view_location,
            timer_wait_stabilize,
            action_clusters,
        ]
    )

    return seq_root
