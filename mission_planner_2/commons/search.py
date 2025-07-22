from typing import List

import numpy as np
import py_trees
import py_trees_ros
from bb_perception_msgs.srv import ClusterTf
from geometry_msgs.msg import PoseStamped

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import (
    create_clustering_goal,
    create_clustering_request,
    create_stamped_pose,
)
from mission_planner_2.trees.auv.goto import goto

_BASE_LINK_FRAME = "auv4/base_link_ned"


def _to_top_left(f: float, l: float, start_xy: np.ndarray) -> np.ndarray:
    """
    Convert forward and left distances to a 2D vector.
    Think of start_xy as some relative position/vector from the origin.
    We then apply the forward and left offsets to this position to get the position vector from the origin.
    """
    return np.array(
        [
            -start_xy[0] + f,
            -start_xy[1] - l,
        ],
        dtype=float,
    )


def _gen_square(
    f: float, b: float, l: float, r: float, start_xy: np.ndarray
) -> np.ndarray:
    top_left = _to_top_left(f, l, start_xy)
    btm_left = np.array((-f - b, 0))
    btm_right = np.array((0, l + r))
    top_right = np.array((f + b, 0))
    return np.array(
        [
            top_left,
            btm_left,
            btm_right,
            top_right,
        ],
        dtype=float,
    )


def _generate_layered_square_search_bot_pattern(
    fwd: float,
    back: float,
    left: float,
    right: float,
    num_squares: int,
    offset_coeff: float = 0.2,
) -> list:
    """
    Generate a layered square search pattern.
    All returned points are defined relative to base_link, suitable to be used with goto.NFromConstant.

    Args:
        fwd (float): Forward distance of the square (this is for first layer offset will be applied with layer_num * offset where layer_num starts from 0 up to num_squares - 1).
        back (float): Backward distance of the square.
        left (float): Left distance of the square.
        right (float): Right distance of the square.
        num_squares (int): Number of squares/layers to generate in the pattern.
        offset_coeff (float): Coefficient to determine the distance between squares.
            The distance between squares is `i * offset_coeff` where `i` is the square index (1-indexed).
    Returns:
        list: A list of PoseStamped objects representing the search pattern.
    """
    output_points = []
    end = np.zeros_like((2, 1), dtype=float)
    for i in range(0, num_squares):
        offset = i * offset_coeff
        points = _gen_square(
            fwd + offset, back + offset, left + offset, right + offset, end
        )
        output_points.append(points)
        end = np.sum(points, axis=0) + end

    output_points = np.concatenate(output_points, axis=0)
    return [
        create_stamped_pose(_BASE_LINK_FRAME, position_x=point[0], position_y=point[1])
        for point in output_points
    ]


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
    cluster_node_start,
    cluster_node_end,
    wait_between_moves_sec: float = 5.0,
):
    root = py_trees.composites.Sequence(
        name="Search seq (bot cam)",
        memory=True,
    )

    goto_search_pattern = goto.NFromConstant(
        name=f"Goto search pattern",
        poses=poses,
        wait_between_moves_sec=wait_between_moves_sec,
        ignore_depth=True,  # TODO: never tested in the pool only in sim
        specified_heading=True,  # dont need to face dir for this search
    )

    root.add_children(
        [
            cluster_node_start,
            goto_search_pattern,
            cluster_node_end,
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

    cluster_node_start = py_trees_ros.service_clients.FromConstant(
        name="Cluster search",
        service_type=ClusterTf,
        service_name="/auv4/cluster_tfs_srv",
        service_request=create_clustering_request(
            enabled=True,
            in_children=object_frame,
            out_children=object_frame_clustered,
            persistent=False,
        ),
    )

    cluster_node_stop = py_trees_ros.service_clients.FromConstant(
        name="Cluster search stop",
        service_type=ClusterTf,
        service_name="/auv4/cluster_tfs_srv",
        service_request=create_clustering_request(
            enabled=False,
            persistent=False,
            in_children=object_frame,
            out_children=object_frame_clustered,
        ),
    )

    root = _create_search_bot_root(
        poses,
        cluster_node_start=cluster_node_start,
        cluster_node_end=cluster_node_stop,
        wait_between_moves_sec=wait_between_moves,
    )

    return root


def create_search_bot_layered_square_root(
    fwd: float,
    back: float,
    left: float,
    right: float,
    num_squares: int,
    object_frame: str,
    object_frame_clustered: str,
    offset_coeff: float = 0.2,
    wait_between_moves: float = 5.0,
):
    poses = _generate_layered_square_search_bot_pattern(
        fwd, back, left, right, num_squares, offset_coeff
    )

    cluster_node_start = py_trees_ros.service_clients.FromConstant(
        name="Cluster search",
        service_type=ClusterTf,
        service_name="/auv4/cluster_tfs_srv",
        service_request=create_clustering_request(
            enabled=True,
            in_children=object_frame,
            out_children=object_frame_clustered,
            persistent=False,
        ),
    )

    cluster_node_stop = py_trees_ros.service_clients.FromConstant(
        name="Cluster search stop",
        service_type=ClusterTf,
        service_name="/auv4/cluster_tfs_srv",
        service_request=create_clustering_request(
            enabled=False,
            persistent=False,
            in_children=object_frame,
            out_children=object_frame_clustered,
        ),
    )

    root = _create_search_bot_root(
        poses,
        cluster_node_start=cluster_node_start,
        cluster_node_end=cluster_node_stop,
        wait_between_moves_sec=wait_between_moves,
    )

    # cluster start finish one layer stop then go next
    # TODO: if u want to add logic to early stop add into the root children
    # root_for_samuel = py_trees.composites.Sequence(
    #     name="Search seq (bot cam) with layers",
    #     memory=True,
    # )
    # children: List[py_trees.behaviour.Behaviour] = [
    #     py_trees.composites.Sequence(
    #         name=f"Search layer {i + 1}",
    #         memory=True,
    #         children=[
    #             _create_search_bot_root(
    #                 poses[i],
    #                 cluster_node_start=py_trees_ros.service_clients.FromConstant(
    #                     name="Cluster search",
    #                     service_type=ClusterTf,
    #                     service_name="/auv4/cluster_tfs_srv",
    #                     service_request=create_clustering_request(
    #                         enabled=True,
    #                         in_children=object_frame,
    #                         out_children=object_frame_clustered,
    #                         persistent=False,
    #                     ),
    #                 ),
    #                 cluster_node_end=py_trees_ros.service_clients.FromConstant(
    #                     name="Cluster search stop",
    #                     service_type=ClusterTf,
    #                     service_name="/auv4/cluster_tfs_srv",
    #                     service_request=create_clustering_request(
    #                         enabled=False,
    #                         persistent=False,
    #                         in_children=object_frame,
    #                         out_children=object_frame_clustered,
    #                     ),
    #                 ),
    #                 wait_between_moves_sec=wait_between_moves,
    #             )
    #         ],
    #     )
    #     for i in range(len(poses))
    # ]
    # root_for_samuel.add_children(children)

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
    # TODO: cant use namspace andfk method?
    enable_request_key = "/bot_search/service_request"
    disable_request_key = "/bot_search/service_request_stop"

    root = py_trees.composites.Sequence(
        name="Search seq (bot cam)",
        memory=True,
    )

    dynamic_set_start_req = DynamicSetBlackboard(
        name="Dynamic set cluster goal",
        key=[object_frame_key, object_frame_clustered_key],
        update_key=enable_request_key,
        overwrite=True,
        func=lambda frame, frame_clustered: create_clustering_request(
            enabled=True,
            in_children=frame,
            out_children=frame_clustered,
            persistent=False,
        ),
    )

    dynamic_set_end_req = DynamicSetBlackboard(
        name="Dynamic set cluster stop",
        key=[object_frame_key, object_frame_clustered_key],
        update_key=disable_request_key,
        overwrite=True,
        func=lambda frame, frame_clustered: create_clustering_request(
            enabled=False,
            persistent=False,
            in_children=frame,
            out_children=frame_clustered,
        ),
    )

    cluster_node_start = py_trees_ros.service_clients.FromBlackboard(
        name="Cluster search",
        service_type=ClusterTf,
        service_name="/auv4/cluster_tfs_srv",
        key_request=enable_request_key,
    )

    cluster_node_end = py_trees_ros.service_clients.FromBlackboard(
        name="Cluster search stop",
        service_type=ClusterTf,
        service_name="/auv4/cluster_tfs_srv",
        key_request=disable_request_key,
    )

    seq_search = _create_search_bot_root(
        poses,
        cluster_node_start=cluster_node_start,
        cluster_node_end=cluster_node_end,
        wait_between_moves_sec=wait_between_moves,
    )

    root.add_children(
        [
            dynamic_set_start_req,
            dynamic_set_end_req,
            seq_search,
        ]
    )

    return root


def create_search_front_root(
    object_frame: str,
    object_frame_clustered: str,
    wait_between_moves: float = 5.0,
):
    root = py_trees.composites.Sequence(
        name="Search seq (front cam)",
        memory=True,
    )

    goto_yaw = goto.NFromConstant(
        name="Goto search pattern",
        poses=[
            create_stamped_pose(_BASE_LINK_FRAME, yaw=15.0),
            create_stamped_pose(_BASE_LINK_FRAME, yaw=15.0),
            create_stamped_pose(_BASE_LINK_FRAME, yaw=-45.0),
            create_stamped_pose(_BASE_LINK_FRAME, yaw=-15.0),
        ],
        wait_between_moves_sec=wait_between_moves,
        ignore_depth=True,
        # specified_heading=True,
    )

    cluster_node_start = py_trees_ros.service_clients.FromConstant(
        name="Cluster search",
        service_type=ClusterTf,
        service_name="/auv4/cluster_tfs_srv",
        service_request=create_clustering_request(
            enabled=True,
            in_children=object_frame,
            out_children=object_frame_clustered,
            persistent=False,
        ),
    )

    cluster_node_stop = py_trees_ros.service_clients.FromConstant(
        name="Cluster search stop",
        service_type=ClusterTf,
        service_name="/auv4/cluster_tfs_srv",
        service_request=create_clustering_request(
            enabled=False,
            persistent=False,
            in_children=object_frame,
            out_children=object_frame_clustered,
        ),
    )

    root.add_children(
        [
            cluster_node_start,
            goto_yaw,
            cluster_node_stop,
        ]
    )

    return root
