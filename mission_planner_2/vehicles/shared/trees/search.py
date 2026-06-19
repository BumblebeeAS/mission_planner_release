from typing import Callable, List, Optional

import numpy as np
import py_trees
import py_trees_ros
from bb_perception_msgs.msg import ClusterPoseResultArray
from bb_perception_msgs.srv import ClusterPosesSrv, ClusterTfSrv
from geometry_msgs.msg import PoseStamped
from py_trees_ros.subscribers import operator
from rclpy.qos import qos_profile_sensor_data

from mission_planner_2.common.util.pose_utils import (
    create_clustering_request,
    create_stamped_pose,
)
from mission_planner_2.vehicles.auv.trees.robosub24.goto import goto
from mission_planner_2.vehicles.shared.trees.blackboard import DynamicSetBlackboard

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
    base_link_frame: str = _BASE_LINK_FRAME,
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
            The distance between squares is `i * offset_coeff` where `i` is the square index (0-indexed).
        base_link_frame (str): Frame used for the generated relative poses.
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
        create_stamped_pose(base_link_frame, position_x=point[0], position_y=point[1])
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
    search_depth: float = 0.3,
):
    root = py_trees.composites.Sequence(
        name="Search seq (bot cam)",
        memory=True,
    )

    goto_search_pattern = goto.NFromConstant(
        name="Goto search pattern",
        poses=poses,
        wait_between_moves_sec=wait_between_moves_sec,
        specified_heading=True,  # dont need to face dir for this search
        # is_relative_movement=True,
        depth_override_value=search_depth,
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
    search_depth: float = 0.3,
):
    poses = _generate_square(fwd, back, left, right)

    cluster_node_start = py_trees_ros.service_clients.FromConstant(
        name="Cluster search",
        service_type=ClusterTfSrv,
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
        service_type=ClusterTfSrv,
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
        search_depth=search_depth,
    )

    return root


def create_homing_search_bot_layered_square_root(
    fwd: float,
    back: float,
    left: float,
    right: float,
    num_squares: int,
    object_frame: str,
    cluster_dist_threshold: float,
    object_frame_clustered: str,
    check_topic: str,
    check_topic_type,
    offset_coeff: float = 0.2,
    wait_between_moves: float = 5.0,
    search_depth: float = 0.3,
    min_cluster_size: int = 2,
):
    # TODO: dont use the cluster node start let people pass in for the hornets to figure out
    cluster_key = "homing_bot_layered_cluster_resp"
    check_topic_sub_key = f"{check_topic}_message"
    poses = _generate_layered_square_search_bot_pattern(
        fwd,
        back,
        left,
        right,
        num_squares,
        offset_coeff,
    )

    root = py_trees.composites.Sequence(
        name="Seq homing search with early stop",
        memory=True,
    )

    def cluster_node_start_func(persistent: bool):
        return py_trees_ros.service_clients.FromConstant(
            name="Cluster search",
            service_type=ClusterTfSrv,
            service_name="/auv4/cluster_tfs_srv",
            service_request=create_clustering_request(
                enabled=True,
                in_children=object_frame,
                out_children=object_frame_clustered,
                persistent=persistent,
                min_cluster_size=min_cluster_size,
            ),
        )

    def cluster_node_stop_func(persistent: bool):
        return py_trees_ros.service_clients.FromConstant(
            name="Cluster search stop",
            service_type=ClusterTfSrv,
            service_name="/auv4/cluster_tfs_srv",
            service_request=create_clustering_request(
                enabled=False,
                persistent=persistent,
                in_children=object_frame,
                out_children=object_frame_clustered,
            ),
            key_response=cluster_key,
        )

    srv_start_cluster = cluster_node_start_func(persistent=False)

    par_search_check = py_trees.composites.Parallel(
        name="Search layered - check par",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )

    goto_search_pattern = goto.NFromConstant(
        name="Goto search pattern",
        poses=poses,
        wait_between_moves_sec=wait_between_moves,
        specified_heading=True,  # dont need to face dir for this search
        depth_override_value=search_depth,
    )

    seq_check_seen = py_trees.composites.Sequence(
        name="Seq sub and check seen something",
        memory=False,
    )

    sub_check_topic = py_trees_ros.subscribers.ToBlackboard(
        name=f"Sub {check_topic}",
        topic_name=check_topic,
        topic_type=check_topic_type,
        qos_profile=qos_profile_sensor_data,
        blackboard_variables={check_topic_sub_key: "data"},
    )

    check_check_ok = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check early stopping",
        check=py_trees.common.ComparisonExpression(
            variable=check_topic_sub_key,
            value=True,
            operator=operator.eq,
        ),
    )

    goto_stationkeep = goto.FromConstant(
        name="goto stationkeep",
        pose=create_stamped_pose(frame_id="auv4/base_link_ned"),
        depth_override_value=search_depth,
    )

    seq_check_seen.add_children(
        [
            sub_check_topic,
            check_check_ok,
        ]
    )

    always_running = py_trees.decorators.FailureIsRunning(
        name="Keep running on failure check",
        child=seq_check_seen,
    )

    par_search_check.add_children(
        [
            goto_search_pattern,
            always_running,
        ]
    )

    seq_stop_search = py_trees.composites.Sequence(
        name="Seq check stop search",
        memory=True,
    )

    srv_stop_cluster = cluster_node_stop_func(persistent=False)

    check_valid_cluster = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check valid cluster",
        check=py_trees.common.ComparisonExpression(
            variable=cluster_key,
            value=cluster_dist_threshold,
            operator=lambda x, y: x.cluster_spread < y or num_squares == 1,
        ),
    )

    seq_stop_search.add_children(
        [
            srv_stop_cluster,
            # check_valid_cluster,
        ]
    )

    root.add_children(
        [
            # srv_start_cluster,
            par_search_check,
            goto_stationkeep,
            # seq_stop_search,
        ]
    )

    retry_homing_seq = py_trees.decorators.Retry(
        name="Retry homing search",
        child=root,
        num_failures=10000,
    )

    return retry_homing_seq


def create_search_bot_layered_square_root(
    fwd: float,
    back: float,
    left: float,
    right: float,
    num_squares: int,
    object_frame: str,
    cluster_dist_threshold: float,
    object_frame_clustered: str,
    offset_coeff: float = 0.2,
    wait_between_moves: float = 5.0,
    search_depth: float = 0.3,
    min_cluster_size: int = 2,
):
    cluster_key = "bot_layered_cluster_resp"

    poses = _generate_layered_square_search_bot_pattern(
        fwd,
        back,
        left,
        right,
        num_squares,
        offset_coeff,
    )

    def cluster_node_start_func(persistent: bool):
        return py_trees_ros.service_clients.FromConstant(
            name="Cluster search",
            service_type=ClusterTfSrv,
            service_name="/auv4/cluster_tfs_srv",
            service_request=create_clustering_request(
                enabled=True,
                in_children=object_frame,
                out_children=object_frame_clustered,
                persistent=persistent,
                min_cluster_size=min_cluster_size,
            ),
        )

    def cluster_node_stop_func(persistent: bool):
        return py_trees_ros.service_clients.FromConstant(
            name="Cluster search stop",
            service_type=ClusterTfSrv,
            service_name="/auv4/cluster_tfs_srv",
            service_request=create_clustering_request(
                enabled=False,
                persistent=persistent,
                in_children=object_frame,
                out_children=object_frame_clustered,
            ),
            key_response=cluster_key,
        )

    def cluster_validate_seq(
        poses: List[PoseStamped],
        cluster_dist_threshold: float,
    ):
        root = py_trees.composites.Sequence(
            name="Seq bot search and validate",
            memory=True,
        )

        seq_search_bot = _create_search_bot_root(
            poses=poses,
            cluster_node_start=cluster_node_start_func(True),
            cluster_node_end=cluster_node_stop_func(True),
            wait_between_moves_sec=wait_between_moves,
            search_depth=search_depth,
        )

        check_valid_cluster = py_trees.behaviours.CheckBlackboardVariableValue(
            name="Check valid cluster",
            check=py_trees.common.ComparisonExpression(
                variable=cluster_key,
                value=cluster_dist_threshold,
                operator=lambda x, y: x.cluster_spread < y or num_squares == 1,
            ),
        )

        root.add_children(
            [
                seq_search_bot,
                check_valid_cluster,
            ]
        )

        return root

    # root = _create_search_bot_root(
    #     poses,
    #     cluster_node_start=cluster_node_start_func(False),
    #     cluster_node_end=cluster_node_stop_func(False),
    #     wait_between_moves_sec=wait_between_moves,
    #     search_depth=search_depth,
    # )

    # cluster start finish one layer stop then go next
    # TODO: if u want to add logic to early stop add into the root children
    root = py_trees.composites.Selector(
        name="Search seq (bot cam) with layers",
        memory=True,
    )

    root.add_children(
        [
            cluster_validate_seq(
                poses=poses[layer_num * 4 : layer_num * 4 + 4],
                cluster_dist_threshold=cluster_dist_threshold,
            )
            for layer_num in range(len(poses) // 4)
        ]
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
    search_depth: float = 0.3,
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
        service_type=ClusterTfSrv,
        service_name="/auv4/cluster_tfs_srv",
        key_request=enable_request_key,
    )

    cluster_node_end = py_trees_ros.service_clients.FromBlackboard(
        name="Cluster search stop",
        service_type=ClusterTfSrv,
        service_name="/auv4/cluster_tfs_srv",
        key_request=disable_request_key,
    )

    seq_search = _create_search_bot_root(
        poses,
        cluster_node_start=cluster_node_start,
        cluster_node_end=cluster_node_end,
        wait_between_moves_sec=wait_between_moves,
        search_depth=search_depth,
    )

    root.add_children(
        [
            dynamic_set_start_req,
            dynamic_set_end_req,
            seq_search,
        ]
    )

    return root


def _gen_yaw_points(max_left: float, max_right: float, s: float) -> List[PoseStamped]:
    """
    Generate yaw points for searching. Floors the max angles by step size s.
    """
    num_points_l = int(max_left // s)
    num_points_r = int(max_right // s)
    points = [create_stamped_pose(_BASE_LINK_FRAME, yaw=s) for _ in range(num_points_r)]

    offset = (num_points_r + 1) * s
    points.append(create_stamped_pose(_BASE_LINK_FRAME, yaw=-offset - s))
    for _ in range(num_points_l - 1):
        points.append(create_stamped_pose(_BASE_LINK_FRAME, yaw=-s))

    return points


def create_search_front_root(
    object_frame: str,
    object_frame_clustered: str,
    yaw_left_deg: float = 30.0,
    yaw_right_deg: float = 30.0,
    step: float = 15.0,
    wait_between_moves: float = 5.0,
    search_depth: float = 0.3,
):
    """Search front yaw angles will be done in multiple of 15 degrees."""
    root = py_trees.composites.Sequence(
        name="Search seq (front cam)",
        memory=True,
    )

    points = _gen_yaw_points(
        max_left=yaw_left_deg,
        max_right=yaw_right_deg,
        s=step,
    )

    goto_yaw = goto.NFromConstant(
        name="Goto search pattern",
        poses=points,
        wait_between_moves_sec=wait_between_moves,
        is_relative_movement=True,
        # specified_heading=True,
        depth_override_value=search_depth,
    )

    cluster_node_start = py_trees_ros.service_clients.FromConstant(
        name="Cluster search",
        service_type=ClusterTfSrv,
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
        service_type=ClusterTfSrv,
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


def create_new_search_bot_layered_square_root(
    fwd: float,
    back: float,
    left: float,
    right: float,
    num_squares: int,
    decision_func: Callable[[ClusterPoseResultArray], str],
    cluster_request_key: str,
    goto_n_from_constant_cls: Callable[..., py_trees.behaviour.Behaviour],
    goto_from_blackboard_cls: Callable[..., py_trees.behaviour.Behaviour],
    base_link_frame: str,
    goto_from_constant_cls: Optional[Callable[..., py_trees.behaviour.Behaviour]] = None,
    relocate_frame: Optional[str] = None,
    spike_topic: str = "/cluster_pose_results",
    cluster_service_name: str = "/cluster_poses_srv",
    offset_coeff: float = 0.2,
    wait_between_moves: float = 5.0,
    search_depth: float = 0.3,
    enable_spike_search: bool = True,
):
    """Create a layered search driven by periodic pose-clustering results.

    The request stored at ``cluster_request_key`` starts the clustering service.
    ``decision_func`` maps each result array to ``"exit"``, ``"relocate"``, or
    ``"continue"``. Goto behaviour classes are injected so this shared builder
    remains vehicle agnostic.
    """
    spike_msg_key = "spike_status_msg"
    decision_key = "spike_decision"
    relocate_pose_key = "spike_relocate_pose"

    def _create_cluster_start_service():
        return py_trees_ros.service_clients.FromBlackboard(
            name="Start spike cluster",
            service_type=ClusterPosesSrv,
            service_name=cluster_service_name,
            key_request=cluster_request_key,
        )

    def _create_cluster_stop_service():
        stop_request = ClusterPosesSrv.Request()
        stop_request.enabled = False
        return py_trees_ros.service_clients.FromConstant(
            name="Stop spike cluster",
            service_type=ClusterPosesSrv,
            service_name=cluster_service_name,
            service_request=stop_request,
        )

    poses = _generate_layered_square_search_bot_pattern(
        fwd,
        back,
        left,
        right,
        num_squares,
        offset_coeff=offset_coeff,
        base_link_frame=base_link_frame,
    )

    set_init_decision = py_trees.behaviours.SetBlackboardVariable(
        name="Init decision",
        variable_name=decision_key,
        variable_value="continue",
        overwrite=True,
    )

    check_decision = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check decision == continue",
        check=py_trees.common.ComparisonExpression(
            variable=decision_key,
            value="continue",
            operator=operator.eq,
        ),
    )

    goto_search_pattern = goto_n_from_constant_cls(
        name="Goto search pattern",
        poses=poses,
        wait_between_moves_sec=wait_between_moves,
        specified_heading=True,
        depth_override_value=search_depth,
    )

    seq_search_branch = py_trees.composites.Sequence(
        name="Seq search pattern",
        memory=True,
    )
    seq_search_branch.add_children([check_decision, goto_search_pattern])

    if enable_spike_search:
        loop_body = py_trees.composites.Sequence(
            name="Search-or-relocate iteration",
            memory=True,
        )
        retry_loop = py_trees.decorators.Retry(
            name="Repeat after relocate",
            child=loop_body,
            num_failures=10000,
        )
        search_par = py_trees.composites.Parallel(
            name="Search layered - spike par",
            policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
        )
        seq_spike = py_trees.composites.Sequence(
            name="Seq_spike_react",
            memory=False,
        )
        failure_running_search = py_trees.decorators.FailureIsRunning(
            name="Failure is running search",
            child=seq_search_branch,
        )
        sub_spike = py_trees_ros.subscribers.ToBlackboard(
            name="Sub cluster results",
            topic_name=spike_topic,
            topic_type=ClusterPoseResultArray,
            qos_profile=qos_profile_sensor_data,
            blackboard_variables={spike_msg_key: None},
        )
        decide = DynamicSetBlackboard(
            name="Run decision func",
            key=spike_msg_key,
            update_key=decision_key,
            overwrite=True,
            func=decision_func,
        )
        extract_pose = DynamicSetBlackboard(
            name="Extract best-cluster pose",
            key=spike_msg_key,
            update_key=relocate_pose_key,
            overwrite=True,
            func=lambda msg: (
                PoseStamped(
                    header=msg.header,
                    pose=msg.results[0].clustered_pose,
                )
                if msg.results
                else PoseStamped(header=msg.header)
            ),
        )

        decide_branch = py_trees.composites.Selector(
            name="Decide branch",
            memory=False,
        )
        check_exit = py_trees.behaviours.CheckBlackboardVariableValue(
            name="Decision == exit",
            check=py_trees.common.ComparisonExpression(
                variable=decision_key,
                value="exit",
                operator=operator.eq,
            ),
        )
        goto_exit = goto_from_blackboard_cls(
            name="Goto exit pose",
            pose_key=relocate_pose_key,
            depth_override_value=search_depth,
        )
        decide_branch_children = [check_exit]

        if relocate_frame is not None and goto_from_constant_cls is not None:
            seq_relocate = py_trees.composites.Sequence(
                name="Seq_relocate",
                memory=True,
            )
            check_relocate = py_trees.behaviours.CheckBlackboardVariableValue(
                name="Decision == relocate",
                check=py_trees.common.ComparisonExpression(
                    variable=decision_key,
                    value="relocate",
                    operator=operator.eq,
                ),
            )
            goto_relocate = goto_from_constant_cls(
                name="Goto spike pose",
                pose=create_stamped_pose(relocate_frame),
                depth_override_value=search_depth,
            )
            seq_relocate.add_children([check_relocate, goto_relocate])
            decide_branch_children.append(seq_relocate)

        decide_branch_children.append(
            py_trees.behaviours.Running(name="Continue current search")
        )
        decide_branch.add_children(decide_branch_children)
        seq_spike.add_children(
            [
                sub_spike,
                decide,
                extract_pose,
                decide_branch,
            ]
        )
        search_par.add_children([failure_running_search, seq_spike])

        seq_exit = py_trees.composites.Sequence(
            name="Seq exit",
            memory=True,
        )
        check_was_exit = py_trees.behaviours.CheckBlackboardVariableValue(
            name="Was exit?",
            check=py_trees.common.ComparisonExpression(
                variable=decision_key,
                value="exit",
                operator=operator.eq,
            ),
        )
        seq_exit.add_children([check_was_exit, goto_exit])
        loop_body.add_children([search_par, seq_exit])
        main_logic = retry_loop
    else:
        main_logic = seq_search_branch

    cleanup_selector = py_trees.composites.Selector(
        name="Search with cluster cleanup",
        memory=True,
    )
    success_then_stop = py_trees.composites.Sequence(
        name="Search then stop cluster",
        memory=True,
    )
    success_then_stop.add_children(
        [
            main_logic,
            _create_cluster_stop_service(),
        ]
    )
    fail_then_stop = py_trees.composites.Sequence(
        name="Stop cluster then propagate failure",
        memory=True,
    )
    fail_then_stop.add_children(
        [
            _create_cluster_stop_service(),
            py_trees.behaviours.Failure(name="Propagate search failure"),
        ]
    )
    cleanup_selector.add_children([success_then_stop, fail_then_stop])

    root = py_trees.composites.Sequence(
        name="Spike search root",
        memory=True,
    )
    root.add_children(
        [
            set_init_decision,
            _create_cluster_start_service(),
            cleanup_selector,
        ]
    )
    return root
