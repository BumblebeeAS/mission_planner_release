from typing import Callable, Optional

import numpy as np
import py_trees
import py_trees_ros
from bb_perception_msgs.msg import ClusterPoseResultArray
from bb_perception_msgs.srv import ClusterPosesSrv
from geometry_msgs.msg import PoseStamped
from py_trees_ros.subscribers import operator
from rclpy.qos import qos_profile_sensor_data

from mission_planner_2.common.util.pose_utils import create_stamped_pose
from mission_planner_2.vehicles.shared.trees.blackboard import (
    DynamicSetBlackboard,
)


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
    base_link_frame: str,
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
        base_link_frame (str): The vehicle base-link frame the poses are expressed in.
        offset_coeff (float): Coefficient to determine the distance between squares.
            The distance between squares is `i * offset_coeff` where `i` is the square index (0-indexed).
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
    """Layered square search driven by cluster_poses_service_node results.

    `cluster_request_key` is a blackboard key holding the full
    `ClusterPosesSrv.Request` (with enabled=True and all per-call config:
    topics, frame IDs, clustering params). To get periodic feedback the request
    must set `cluster_interval > 0`, which makes the service publish a
    `ClusterPoseResultArray` on `spike_topic` every interval. The caller must
    set this key before the root ticks. The stop call uses a fixed
    `enabled=False` request.

    The goto behaviours are INJECTED so this shared builder stays vehicle
    agnostic:
      - `goto_n_from_constant_cls`: builds the multi-pose search-pattern goto.
        Called as ``cls(name, poses=..., wait_between_moves_sec=...,
        specified_heading=True, depth_override_value=...)``.
      - `goto_from_blackboard_cls`: builds the extracted-pose goto. Called as
        ``cls(name, pose_key=..., depth_override_value=...)``.
      - `goto_from_constant_cls` (optional): builds the relocate goto. Called as
        ``cls(name, pose=..., depth_override_value=...)``. Required only when a
        `relocate_frame` is supplied (otherwise the "relocate" decision path is
        omitted).
    `base_link_frame` is the frame the generated square pattern is expressed in.

    When `enable_spike_search` is True (default), branch 1 runs the goto search
    pattern and branch 2 subscribes to the cluster-result topic, runs
    `decision_func` on each `ClusterPoseResultArray`, and reacts:
      - "exit"     -> parallel exits, root SUCCESS (good cluster found); the
                       extracted best-cluster pose is used by `goto_exit`.
      - "relocate" -> goto the static `relocate_frame`, then the whole
                       parallel restarts from the new base_link (Retry loop).
      - anything else -> branch stays RUNNING, goto pattern continues.

    When `enable_spike_search` is False, only the seq_search branch runs (the
    plain goto search pattern); the root fails if the pattern fails.

    Clustering is started before the search and stopped on every exit path
    (success or failure of the inner search logic).

    `decision_func` should dedupe on `msg.header.stamp` or it'll re-trigger
    on the same snapshot every tick.
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
        base_link_frame,
        offset_coeff,
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
    seq_search_branch.add_children(
        [
            check_decision,
            goto_search_pattern,
        ]
    )

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

        # ClusterPoseResultArray is published with the default (RELIABLE) QoS,
        # so subscribe RELIABLE (depth 10) — sensor_data (BEST_EFFORT) would be
        # QoS-incompatible and drop every message.
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

        # ClusterPoseResult.clustered_pose is a bare Pose; the frame_id + stamp
        # live once on the array header. Build the PoseStamped from that header
        # (no TF lookup). Guard the empty-results case so periodic empty ticks
        # don't IndexError.
        extract_pose = DynamicSetBlackboard(
            name="Extract best-cluster pose",
            key=spike_msg_key,
            update_key=relocate_pose_key,
            overwrite=True,
            func=lambda m: (
                PoseStamped(header=m.header, pose=m.results[0].clustered_pose)
                if m.results
                else PoseStamped(header=m.header)
            ),
        )

        decide_branch = py_trees.composites.Selector(
            name="Decide branch",
            memory=False,
        )

        seq_exit = py_trees.composites.Sequence(
            name="Seq exit",
            memory=True,
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

        # The relocate path is optional: it requires both a static frame to
        # relocate to and a concrete FromConstant goto to drive there. Skip it
        # entirely when the caller does not configure relocation.
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

            seq_relocate.add_children(
                [
                    check_relocate,
                    goto_relocate,
                ]
            )
            decide_branch_children.append(seq_relocate)

        continue_run = py_trees.behaviours.Running(
            name="Continue current search",
        )
        decide_branch_children.append(continue_run)

        decide_branch.add_children(decide_branch_children)

        seq_spike.add_children(
            [
                sub_spike,
                decide,
                extract_pose,
                decide_branch,
            ]
        )

        search_par.add_children(
            [
                failure_running_search,
                seq_spike,
            ]
        )

        check_was_exit = py_trees.behaviours.CheckBlackboardVariableValue(
            name="Was exit?",
            check=py_trees.common.ComparisonExpression(
                variable=decision_key,
                value="exit",
                operator=operator.eq,
            ),
        )

        seq_exit.add_children(
            [
                check_was_exit,
                goto_exit,
            ]
        )

        loop_body.add_children(
            [
                search_par,
                seq_exit,
            ]
        )

        main_logic = retry_loop
    else:
        main_logic = seq_search_branch

    # Guarantee `stop_cluster` runs on every exit (success and failure of
    # main_logic) by wrapping main_logic in a Selector with success/failure
    # paths that both stop clustering before propagating their status.
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

    cleanup_selector.add_children(
        [
            success_then_stop,
            fail_then_stop,
        ]
    )

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
