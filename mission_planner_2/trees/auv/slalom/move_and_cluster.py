import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from mission_planner_2.commons.namespace_utils import (
    generate_namespace,
    full_key_generator,
)
from mission_planner_2.commons.pose_utils import (
    create_stamped_pose,
    create_clustering_goal,
    create_clustering_goals
)
from mission_planner_2.trees.auv.goto import goto
from geometry_msgs.msg import PoseStamped

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

"""
General Idea:
Move to defined location
Stabilise
Send clustering goal
"""

######################### UPDATE CONSTANTS HERE #########################
CLUSTERING_DURATION = 20
STABILIZE_CONTROLS_DURATION = 10
BASE_LINK_FRAME = "auv4/base_link_ned"
#########################################################################


def create_move_and_cluster_root(
        pose_stamped: PoseStamped,
        in_children: list[str],
        out_children: list[str],
        out_parent: str = "world_ned"
    ):
    """
    Create the root of the move and cluster tree.
    Presumes that out_parent is "world_ned"

    Move to a specified location, wait for the controls to stabilize,
    and then perform clustering in parallel on the specified child frames.

    Args:
        pose_stamped: PoseStamped object defining the target location to move to.
        in_children: List of child frames to cluster.
        out_parent: Parent frame for the output clustering frame.
        out_children: List of child frames to output after clustering.
   """
    seq_root = py_trees.composites.Sequence(
        name="Move and cluster",
        memory=True,
    )    

    goto_view_location = goto.FromConstant(
        name="Goto view location",
        parent_namespace=NAMESPACE,
        pose=pose_stamped
    )

    timer_wait_stabilize = py_trees.timers.Timer(
        name="Stabilize before clustering",
        duration=STABILIZE_CONTROLS_DURATION,
    )

    # Multiple clustering goals
    action_clusters = py_trees_ros.action_clients.FromConstant(
        name="Cluster transforms",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goals(
            in_children=in_children,
            out_children=out_children,
            out_parent=out_parent,
            duration=CLUSTERING_DURATION,
            cache_size=1000, # To adjust
            persistent=True, # Reuse caches
        )
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
