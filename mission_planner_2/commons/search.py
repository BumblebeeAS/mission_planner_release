from typing import List

import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from geometry_msgs.msg import PoseStamped

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
)
from mission_planner_2.trees.auv.goto import goto

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)
_BASE_LINK_FRAME = "auv4/base_link_ned"


def _generate_square(fwd, back, left, right):
    """Generate a square pattern with the given dimensions."""
    return [
        # create_stamped_pose(_BASE_LINK_FRAME, 0, 0, 0),  # center of the square
        create_stamped_pose(
            _BASE_LINK_FRAME, position_x=fwd, position_y=-left
        ),  # top left
        create_stamped_pose(_BASE_LINK_FRAME, position_x=-(fwd + back)),  # bottom left,
        create_stamped_pose(_BASE_LINK_FRAME, position_y=left + right),  # bottom right
        create_stamped_pose(_BASE_LINK_FRAME, position_x=fwd + back),
    ]


def _create_search_bot_root(
    poses: List[PoseStamped],
    cluster_node,
    wait_between_moves_sec: float = 5.0,
):

    goto_search_pattern = goto.NFromConstant(
        name=f"Goto search pattern",
        poses=poses,
        wait_between_moves_sec=wait_between_moves_sec,
        ignore_depth=True,  # TODO: never tested in the pool only in sim
        specified_heading=True,  # dont need to face dir for this search
    )

    root = py_trees.composites.Parallel(
        name="Search seq (constant)",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(  # TODO: never tried before
            children=[goto_search_pattern]
        ),
    )

    root.add_children(
        [
            goto_search_pattern,
            cluster_node,
        ]
    )

    return root


def create_search_bot_constant_root(
    fwd: float,
    back: float,
    left: float,
    right: float,
    object_frame: str,
    object_frame_clustered: str,
    wait_between_moves: float = 5.0,
):
    poses = _generate_square(fwd, back, left, right)

    cluster_node = py_trees_ros.action_clients.FromConstant(
        name="Cluster search",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=object_frame,
            out_children=object_frame_clustered,
            use_cache=False,
            persistent=False,
            duration=1e6,
        ),
    )

    root = _create_search_bot_root(
        poses, cluster_node, wait_between_moves_sec=wait_between_moves
    )

    return root


def create_search_bot_bb_root(
    fwd: float,
    back: float,
    left: float,
    right: float,
    object_frame_key: str,
    object_frame_clustered_key: str,
    wait_between_moves: float = 5.0,
):
    poses = _generate_square(fwd, back, left, right)
    goal_key = fk("action_goal")

    root = py_trees.composites.Sequence(
        name="Search seq (bot cam)",
        memory=True,
    )

    dynamic_set_goal = DynamicSetBlackboard(
        name="Dynamic set cluster goal",
        key=[object_frame_key, object_frame_clustered_key],
        update_key=goal_key,
        overwrite=True,
        func=lambda frame, frame_clustered: create_clustering_goal(
            in_children=frame,
            out_children=frame_clustered,
            use_cache=False,
            persistent=False,
            duration=1e6,
        ),
    )

    cluster_node = py_trees_ros.action_clients.FromBlackboard(
        name="Cluster search",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        key=goal_key,
    )

    seq_search = _create_search_bot_root(
        poses,
        cluster_node,
        wait_between_moves_sec=wait_between_moves,
    )

    root.add_children(
        [
            dynamic_set_goal,
            seq_search,
        ]
    )

    return root
