import operator
from typing import Literal

import numpy as np
import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from geometry_msgs.msg import PoseStamped, TransformStamped
from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
)
from mission_planner_2.commons.tf_checker import create_tf_checker_from_constant_root
from mission_planner_2.trees.auv.goto import goto

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

########################## UPDATE CONSTANTS HERE #########################
# TODO: i think this was for non waypoint ver if dn can remove
TRANSFORM_CHECK_TIMEOUT = 5.0
CHANNEL_PAIR_ZERO_FRAME_RECLUSTERED = "slalom/reclustered"
CHANNEL_PAIR_ONE_FRAME_RECLUSTERED = "slalom/dummy/one"
CHANNEL_PAIR_TWO_FRAME_RECLUSTERED = "slalom/dummy/two"
RECLUSTER_DURATION = 5
SWEEP_RECLUSTER_DURATION = 10
#########################################################################

_MISSING_LAYER_KEY = fk("missing_layer")  # key for missing layer
_POSE_LIST_KEY = fk("pose_list")
_LAYER_0_POSE = fk("layer_0_pose")
_LAYER_TO_LAYER_TF_KEY = fk("layer_tf")
_LAYER_TO_LAYER_POSE_KEY = fk("layer_pose")


def _recluster_and_goto_sequence(
    current_frame: str,
    next_frame: str,
    clustering_in_children: list[str],
    is_left: bool = True,
    missing_layers: int = 1,
):
    """
    Creates a recluster and goto sequence that:
    1. Gets transform from current to next frame
    2. Creates pose for yaw alignment
    3. Yaws towards next layer
    4. Reclusters the frames
    5. Goto to the reclustered position
    """
    side = "left" if is_left else "right"
    current_frame_with_side = f"{current_frame}/{side}"
    next_frame_with_side = f"{next_frame}/{side}"

    seq_recluster_goto = py_trees.composites.Sequence(
        name=f"Recluster and goto ({side}, missing layers: {missing_layers}) from {current_frame} to {next_frame}",
        memory=True,
    )

    get_current_to_next_tf = create_tf_checker_from_constant_root(
        start_frames=[current_frame_with_side],
        end_frames=[next_frame_with_side],
        timeout=TRANSFORM_CHECK_TIMEOUT,
        update_keys=[_LAYER_TO_LAYER_TF_KEY],
    )

    create_current_to_next_pose = DynamicSetBlackboard(
        name=f"Create yaw pose ({side}, missing layers: {missing_layers})",
        key=_LAYER_TO_LAYER_TF_KEY,
        update_key=_LAYER_TO_LAYER_POSE_KEY,
        overwrite=True,
        func=lambda tf: create_yawed_pose(frame_id=current_frame_with_side, tf=tf),
    )

    goto_yaw_towards_next = goto.FromBlackboard(
        name=f"Yaw towards next layer ({side}, missing layers: {missing_layers})",
        pose_key=_LAYER_TO_LAYER_POSE_KEY,
    )

    recluster_action = py_trees_ros.action_clients.FromConstant(
        name=f"Recluster transforms ({side}, missing layers: {missing_layers})",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf_multi",
        action_goal=create_clustering_goal(
            in_children=clustering_in_children,
            out_children=[
                CHANNEL_PAIR_ZERO_FRAME_RECLUSTERED,
                CHANNEL_PAIR_ONE_FRAME_RECLUSTERED,
                CHANNEL_PAIR_TWO_FRAME_RECLUSTERED,
            ],
            duration=RECLUSTER_DURATION,
        ),
    )

    goto_reclustered = goto.FromConstant(
        name=f"Goto reclustered next layer ({side}, missing layers: {missing_layers})",
        pose=create_stamped_pose(f"{CHANNEL_PAIR_ZERO_FRAME_RECLUSTERED}/{side}"),
        specified_heading=False,
    )

    seq_recluster_goto.add_children(
        [
            get_current_to_next_tf,
            create_current_to_next_pose,
            goto_yaw_towards_next,
            recluster_action,
            goto_reclustered,
        ]
    )

    return seq_recluster_goto


def _sweep_and_goto_sequence(
    clustering_in_children: list[str],
    is_left: bool = True,
    sweep_angle_degrees: float = 45.0,
    missing_layers: int = 1,
):
    """
    Creates a sweep and goto sequence that:
    1. Yaws 45 degrees (default) to the left in base_link
    2. Yaws 45 degrees (default) to the right in base_link
    3. Reclusters in parallel with the sweeping
    4. Goes to the zero cluster
    """
    side = "left" if is_left else "right"

    seq_sweep_goto = py_trees.composites.Sequence(
        name=f"Sweep and goto sequence ({side}, missing layers: {missing_layers})",
        memory=True,
    )

    par_sweep_recluster = py_trees.composites.Parallel(
        name=f"Parallel sweep and recluster ({side}, missing layers: {missing_layers})",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )

    sweep_poses = [
        create_stamped_pose(
            "auv4/base_link_ned", yaw=-sweep_angle_degrees, use_radians=False
        ),
        create_stamped_pose(
            frame_id="auv4/base_link_ned", yaw=sweep_angle_degrees, use_radians=False
        ),
    ]

    goto_sweep = goto.NFromConstant(
        name=f"Sweep left then right ({side}, missing layers: {missing_layers})",
        poses=sweep_poses,
        wait_between_moves_sec=1.0,
    )

    recluster_action = py_trees_ros.action_clients.FromConstant(
        name=f"Recluster during sweep ({side}, missing layers: {missing_layers})",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf_multi",
        action_goal=create_clustering_goal(
            in_children=clustering_in_children,
            out_children=[
                CHANNEL_PAIR_ZERO_FRAME_RECLUSTERED,
                CHANNEL_PAIR_ONE_FRAME_RECLUSTERED,
                CHANNEL_PAIR_TWO_FRAME_RECLUSTERED,
            ],
            duration=SWEEP_RECLUSTER_DURATION,
        ),
    )

    par_sweep_recluster.add_children(
        [
            goto_sweep,
            recluster_action,
        ]
    )

    goto_reclustered_zero = goto.FromConstant(
        name=f"Goto reclustered zero cluster ({side}, missing layers: {missing_layers})",
        pose=create_stamped_pose(f"{CHANNEL_PAIR_ZERO_FRAME_RECLUSTERED}/{side}"),
        specified_heading=False,
    )

    seq_sweep_goto.add_children(
        [
            par_sweep_recluster,
            goto_reclustered_zero,
        ]
    )

    return seq_sweep_goto


def create_yawed_pose(frame_id: str, tf: TransformStamped) -> PoseStamped:
    # _, _, y = euler_from_quaternion(
    #     [*operator.attrgetter("x", "y", "z", "w")(tf.transform.rotation)]
    # )
    yaw = np.arctan2(tf.transform.translation.y, tf.transform.translation.x)
    return create_stamped_pose(frame_id=frame_id, yaw=yaw, use_radians=True)


def create_channel_movement_zero_root(
    slalom_frame_zero_clustered: str = "slalom_layer_0/clustered",
    slalom_frame_one_clustered: str = "slalom_layer_1/clustered",
    slalom_frame_two_clustered: str = "slalom_layer_2/clustered",
    is_left_key: str = "is_left_key",
    wait_between_moves_sec: float = 4.0,
):
    """
    Zero case: Check if is left, if yes then recluster and goto on layer 0 to layer 1,
    then recluster and goto on reclustered to layer 2. Fallback to right case if not left.
    """
    missing_layers = None
    clustering_in_children = [
        slalom_frame_zero_clustered.split("/")[-2],
        slalom_frame_one_clustered.split("/")[-2],
        slalom_frame_two_clustered.split("/")[-2],
    ]

    root = py_trees.composites.Selector(
        name="Channel Movement: Zero Layers Missing",
        memory=True,
    )

    # Left side sequence
    seq_zero_missing_left = py_trees.composites.Sequence(
        name="Left side sequence: zero layers missing",
        memory=True,
    )

    check_is_left = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check is left (Zero Layers Missing)",
        check=py_trees.common.ComparisonExpression(
            variable=is_left_key,
            value=True,
            operator=operator.eq,
        ),
    )

    goto_layer_0_left = goto.FromConstant(
        name="Goto layer 0 (left): zero layers missing",
        pose=create_stamped_pose(f"{slalom_frame_zero_clustered}/left"),
        specified_heading=False,
    )

    recluster_0_to_1_left = _recluster_and_goto_sequence(
        current_frame=slalom_frame_zero_clustered,
        next_frame=slalom_frame_one_clustered,
        clustering_in_children=clustering_in_children,
        is_left=True,
        missing_layers=missing_layers,
    )

    recluster_reclustered_to_2_left = _recluster_and_goto_sequence(
        current_frame=CHANNEL_PAIR_ZERO_FRAME_RECLUSTERED,
        next_frame=slalom_frame_two_clustered,
        clustering_in_children=clustering_in_children,
        is_left=True,
        missing_layers=missing_layers,
    )

    seq_zero_missing_left.add_children(
        [
            check_is_left,
            goto_layer_0_left,
            recluster_0_to_1_left,
            recluster_reclustered_to_2_left,
        ]
    )

    # Right side sequence (fallback)
    seq_zero_missing_right = py_trees.composites.Sequence(
        name="Right side sequence: zero layers missing",
        memory=True,
    )

    goto_layer_0_right = goto.FromConstant(
        name="Goto layer 0 (right): zero layers missing",
        pose=create_stamped_pose(f"{slalom_frame_zero_clustered}/right"),
        specified_heading=False,
    )

    recluster_0_to_1_right = _recluster_and_goto_sequence(
        current_frame=slalom_frame_zero_clustered,
        next_frame=slalom_frame_one_clustered,
        clustering_in_children=clustering_in_children,
        is_left=False,
        missing_layers=missing_layers,
    )

    recluster_reclustered_to_2_right = _recluster_and_goto_sequence(
        current_frame=CHANNEL_PAIR_ZERO_FRAME_RECLUSTERED,
        next_frame=slalom_frame_two_clustered,
        clustering_in_children=clustering_in_children,
        is_left=False,
        missing_layers=missing_layers,
    )

    seq_zero_missing_right.add_children(
        [
            goto_layer_0_right,
            recluster_0_to_1_right,
            recluster_reclustered_to_2_right,
        ]
    )

    root.add_children(
        [
            seq_zero_missing_left,
            seq_zero_missing_right,
        ]
    )

    return root


def check_tf_dist(
    tf: TransformStamped,
    dist_threshold: float = 3.0,
) -> Literal["one", "two"]:
    delta_x = abs(tf.transform.translation.z)

    return "two" if delta_x < dist_threshold else "one"


def create_channel_movement_layer_one_missing_root(
    slalom_frame_zero_clustered: str = "slalom_layer_0/clustered",
    slalom_frame_one_clustered: str = "slalom_layer_1/clustered",
    slalom_frame_two_clustered: str = "slalom_layer_2/clustered",
    is_left: bool = True,
):
    side = "left" if is_left else "right"

    clustering_in_children = [
        slalom_frame_zero_clustered.split("/")[-2],
        slalom_frame_one_clustered.split("/")[-2],
        slalom_frame_two_clustered.split("/")[-2],
    ]

    seq_missing_layer_one = py_trees.composites.Sequence(
        name=f"Movement sequence for missing 1 layer ({side})", memory=True
    )

    goto_layer_0 = goto.FromConstant(
        name=f"Goto layer 0 ({side}): missing 1 layer",
        pose=create_stamped_pose(f"{slalom_frame_zero_clustered}/{side}"),
        specified_heading=False,
    )

    sweep_and_goto_layer_1 = _sweep_and_goto_sequence(
        clustering_in_children=clustering_in_children,
        is_left=is_left,
        missing_layers=1,
    )

    recluster_reclustered_to_2 = _recluster_and_goto_sequence(
        current_frame=CHANNEL_PAIR_ZERO_FRAME_RECLUSTERED,
        next_frame=slalom_frame_two_clustered,
        clustering_in_children=clustering_in_children,
        is_left=is_left,
        missing_layers=1,
    )

    seq_missing_layer_one.add_children(
        [goto_layer_0, sweep_and_goto_layer_1, recluster_reclustered_to_2]
    )

    return seq_missing_layer_one


def create_channel_movement_layer_two_missing_root(
    slalom_frame_zero_clustered: str = "slalom_layer_0/clustered",
    slalom_frame_one_clustered: str = "slalom_layer_1/clustered",
    slalom_frame_two_clustered: str = "slalom_layer_2/clustered",
    is_left: bool = True,
):
    side = "left" if is_left else "right"

    clustering_in_children = [
        slalom_frame_zero_clustered.split("/")[-2],
        slalom_frame_one_clustered.split("/")[-2],
        slalom_frame_two_clustered.split("/")[-2],
    ]

    # This flow is for when one layer is missing (in this case, layer 2).
    seq_missing_layer_two = py_trees.composites.Sequence(
        name=f"Movement sequence for missing 1 layer ({side})", memory=True
    )

    goto_layer_0 = goto.FromConstant(
        name=f"Goto layer 0 ({side}): missing 1 layer",
        pose=create_stamped_pose(f"{slalom_frame_zero_clustered}/{side}"),
        specified_heading=False,
    )

    recluster_reclustered_to_1 = _recluster_and_goto_sequence(
        current_frame=slalom_frame_zero_clustered,
        next_frame=slalom_frame_one_clustered,
        clustering_in_children=clustering_in_children,
        is_left=is_left,
        missing_layers=1,
    )

    sweep_and_goto_layer_2 = _sweep_and_goto_sequence(
        clustering_in_children=clustering_in_children,
        is_left=is_left,
        missing_layers=1,
    )

    seq_missing_layer_two.add_children(
        [goto_layer_0, recluster_reclustered_to_1, sweep_and_goto_layer_2]
    )

    return seq_missing_layer_two


def create_channel_movement_one_root(
    slalom_frame_zero_clustered: str = "slalom_layer_0/clustered",
    slalom_frame_one_clustered: str = "slalom_layer_1/clustered",
    slalom_frame_two_clustered: str = "slalom_layer_2/clustered",
    is_left_key: str = "is_left_key",
    wait_between_moves_sec: float = 4.0,
):
    seq_channel_movement_one_missing = py_trees.composites.Sequence(
        name="Channel Movement: One Layer Missing",
        memory=True,
    )

    get_layer_0_to_1_tf = create_tf_checker_from_constant_root(
        start_frames=[slalom_frame_zero_clustered],
        end_frames=[slalom_frame_one_clustered],
        timeout=TRANSFORM_CHECK_TIMEOUT,
        update_keys=[_LAYER_TO_LAYER_TF_KEY],
    )

    dynamic_tf_check = DynamicSetBlackboard(
        name="Check distance to determine missing layer",
        key=_LAYER_TO_LAYER_TF_KEY,
        update_key=_MISSING_LAYER_KEY,
        overwrite=True,
        func=lambda x: check_tf_dist(x, dist_threshold=3.0),
    )

    sel_missing_layer_logic = py_trees.composites.Selector(
        name="Select logic for missing layer 1 or 2",
        memory=True,
    )

    # Sequence for when layer 1 is determined to be missing
    seq_handle_missing_layer_one = py_trees.composites.Sequence(
        name="Handle missing layer 1", memory=True
    )

    missing_layer_one_check = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check if layer 1 is missing",
        check=py_trees.common.ComparisonExpression(
            variable=_MISSING_LAYER_KEY,
            value="one",
            operator=lambda x, y: x == y,
        ),
    )

    sel_left_right_missing_one = py_trees.composites.Selector(
        name="Select side for missing layer 1",
        memory=True,
    )

    seq_left_missing_one = py_trees.composites.Sequence(
        name="Left side sequence for missing layer 1",
        memory=True,
    )

    check_is_left_one = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check is left (for missing layer 1)",
        check=py_trees.common.ComparisonExpression(
            variable=is_left_key,
            value=True,
            operator=operator.eq,
        ),
    )

    seq_movement_missing_one_left = create_channel_movement_layer_one_missing_root(
        slalom_frame_zero_clustered=slalom_frame_zero_clustered,
        slalom_frame_one_clustered=slalom_frame_one_clustered,
        slalom_frame_two_clustered=slalom_frame_two_clustered,
        is_left=True,
    )

    seq_left_missing_one.add_children(
        [check_is_left_one, seq_movement_missing_one_left]
    )

    seq_movement_missing_one_right = create_channel_movement_layer_one_missing_root(
        slalom_frame_zero_clustered=slalom_frame_zero_clustered,
        slalom_frame_one_clustered=slalom_frame_one_clustered,
        slalom_frame_two_clustered=slalom_frame_two_clustered,
        is_left=False,
    )

    sel_left_right_missing_one.add_children(
        [seq_left_missing_one, seq_movement_missing_one_right]
    )

    seq_handle_missing_layer_one.add_children(
        [missing_layer_one_check, sel_left_right_missing_one]
    )

    # Selector for when layer 2 is determined to be missing (fallback from layer 1 check)
    sel_handle_missing_layer_two = py_trees.composites.Selector(
        name="Select side for missing layer 2",
        memory=True,
    )

    seq_left_missing_two = py_trees.composites.Sequence(
        name="Left side sequence for missing layer 2",
        memory=True,
    )

    check_is_left_two = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check is left (for missing layer 2)",
        check=py_trees.common.ComparisonExpression(
            variable=is_left_key,
            value=True,
            operator=operator.eq,
        ),
    )

    seq_movement_missing_two_left = create_channel_movement_layer_two_missing_root(
        slalom_frame_zero_clustered=slalom_frame_zero_clustered,
        slalom_frame_one_clustered=slalom_frame_one_clustered,
        slalom_frame_two_clustered=slalom_frame_two_clustered,
        is_left=True,
    )

    seq_left_missing_two.add_children(
        [check_is_left_two, seq_movement_missing_two_left]
    )

    seq_movement_missing_two_right = create_channel_movement_layer_two_missing_root(
        slalom_frame_zero_clustered=slalom_frame_zero_clustered,
        slalom_frame_one_clustered=slalom_frame_one_clustered,
        slalom_frame_two_clustered=slalom_frame_two_clustered,
        is_left=False,
    )

    sel_handle_missing_layer_two.add_children(
        [seq_left_missing_two, seq_movement_missing_two_right]
    )

    sel_missing_layer_logic.add_children(
        [seq_handle_missing_layer_one, sel_handle_missing_layer_two]
    )

    seq_channel_movement_one_missing.add_children(
        [
            get_layer_0_to_1_tf,
            dynamic_tf_check,
            sel_missing_layer_logic,
        ]
    )

    return seq_channel_movement_one_missing


def create_channel_movement_two_root(
    slalom_frame_zero_clustered: str = "slalom_layer_0/clustered",
    slalom_frame_one_clustered: str = "slalom_layer_1/clustered",
    slalom_frame_two_clustered: str = "slalom_layer_2/clustered",
    is_left_key: str = "is_left_key",
    wait_between_moves_sec: float = 4.0,
):
    clustering_in_children = [
        slalom_frame_zero_clustered.split("/")[-2],
        slalom_frame_one_clustered.split("/")[-2],
        slalom_frame_two_clustered.split("/")[-2],
    ]

    sel_channel_movement_two_missing = py_trees.composites.Selector(
        name="Channel Movement: Two Layers Missing",
        memory=True,
    )

    # Left side sequence
    seq_two_missing_left = py_trees.composites.Sequence(
        name="Left side sequence: two layers missing",
        memory=True,
    )

    check_is_left = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check is left (Two Layers Missing)",
        check=py_trees.common.ComparisonExpression(
            variable=is_left_key,
            value=True,
            operator=operator.eq,
        ),
    )

    goto_layer_0_left = goto.FromConstant(
        name="Goto layer 0 (left): two layers missing",
        pose=create_stamped_pose(f"{slalom_frame_zero_clustered}/left"),
        specified_heading=False,
    )

    sweep_and_goto_layer_1_left = _sweep_and_goto_sequence(
        clustering_in_children=clustering_in_children,
        is_left=True,
        missing_layers=2,
    )

    sweep_and_goto_layer_2_left = _sweep_and_goto_sequence(
        clustering_in_children=clustering_in_children,
        is_left=True,
        missing_layers=2,
    )

    seq_two_missing_left.add_children(
        [
            check_is_left,
            goto_layer_0_left,
            sweep_and_goto_layer_1_left,
            sweep_and_goto_layer_2_left,
        ]
    )

    # Right side sequence
    seq_two_missing_right = py_trees.composites.Sequence(
        name="Right side sequence: two layers missing",
        memory=True,
    )

    goto_layer_0_right = goto.FromConstant(
        name="Goto layer 0 (right): two layers missing",
        pose=create_stamped_pose(f"{slalom_frame_zero_clustered}/right"),
        specified_heading=False,
    )

    sweep_and_goto_layer_1_right = _sweep_and_goto_sequence(
        clustering_in_children=clustering_in_children,
        is_left=False,
        missing_layers=2,
    )

    sweep_and_goto_layer_2_right = _sweep_and_goto_sequence(
        clustering_in_children=clustering_in_children,
        is_left=False,
        missing_layers=2,
    )

    seq_two_missing_right.add_children(
        [
            goto_layer_0_right,
            sweep_and_goto_layer_1_right,
            sweep_and_goto_layer_2_right,
        ]
    )

    sel_channel_movement_two_missing.add_children(
        [
            seq_two_missing_left,
            seq_two_missing_right,
        ]
    )

    return sel_channel_movement_two_missing
