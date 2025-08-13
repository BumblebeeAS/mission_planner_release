import operator
from typing import Literal

import numpy as np
import py_trees
from geometry_msgs.msg import PoseStamped, TransformStamped
from mission_planner_2.commons import shared_action_client
from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.node_registry import SharedAction
from mission_planner_2.commons.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
)
from mission_planner_2.commons.tf_checker import create_tf_checker_from_constant_root
from mission_planner_2.trees.auv.goto import goto

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

########################## UPDATE CONSTANTS HERE #########################
BASE_LINK_FRAME = "auv4/base_link_ned"
LAYER_ZERO = "slalom_layer_0"
LAYER_ONE = "slalom_layer_1"
LAYER_TWO = "slalom_layer_2"
LAYER_ZERO_NEAR = "slalom_layer_near_0"
LAYER_ONE_NEAR = "slalom_layer_near_1"
LAYER_TWO_NEAR = "slalom_layer_near_2"

LAYER_ZERO_CLUSTERED = "slalom_layer_0/clustered"
LAYER_ONE_CLUSTERED = "slalom_layer_1/clustered"
LAYER_TWO_CLUSTERED = "slalom_layer_2/clustered"
LAYER_ONE_HARDCODED = "slalom_layer_1/hardcoded"
LAYER_TWO_HARDCODED = "slalom_layer_2/hardcoded"
LAYER_ZERO_TO_ONE_CLUSTERED_TF_KEY = fk("layer_zero_to_one_clustered_tf")
LAYER_ONE_TO_TWO_CLUSTERED_TF_KEY = fk("layer_one_to_two_clustered_tf")
LAYER_ZERO_CLUSTERED_KEY = fk("layer_zero_clustered")
LAYER_ONE_CLUSTERED_KEY = fk("layer_one_clustered")
LAYER_TWO_CLUSTERED_KEY = fk("layer_two_clustered")
IS_VALID_CLUSTERS_KEY = fk("is_valid_clusters")
IS_VALID_RECLUSTERED_KEY = fk("is_valid_reclustered")
POSE_FUNC_KEY = fk("pose_func_key")
IS_LEFT_KEY = "/global/is_left_side"  # Global key for left option or not
LAYER_ZERO_POSE_KEY = fk("layer_zero_pose")
LAYER_ONE_POSE_KEY = fk("layer_one_pose")
LAYER_TWO_POSE_KEY = fk("layer_two_pose")
LAYER_TO_LAYER_TF_KEY = fk("layer_to_layer_tf")
LAYER_TO_LAYER_POSE_KEY = fk("layer_to_layer_pose")
CLUSTERING_IN_CHILDREN = [
    LAYER_ZERO,
    LAYER_ONE,
    LAYER_TWO,
]
CLUSTERING_IN_CHILDREN_NEAR = [
    LAYER_ZERO_NEAR,
    LAYER_ONE_NEAR,
    LAYER_TWO_NEAR,
]
LAYER_ZERO_RECLUSTERED = "slalom/reclustered"
LAYER_ONE_RECLUSTERED_DUMMY = "slalom/dummy/one"
LAYER_TWO_RECLUSTERED_DUMMY = "slalom/dummy/two"
RECLUSTER_DURATION = 5
RECLUSTERED_LAYER_KEY = fk("reclustered_layer")
SWEEP_ANGLE_DEGREES = 30.0
SWEEP_RECLUSTER_DURATION = 5
DEPTH_OVERRIDE_VALUE = 0.9
#########################################################################


def create_yawed_pose(tf: TransformStamped) -> PoseStamped:
    yaw = np.arctan2(tf.transform.translation.x, tf.transform.translation.z)
    print(f"Yaw angle for pose: {yaw} radians")
    return create_stamped_pose(frame_id=BASE_LINK_FRAME, yaw=yaw, use_radians=True)


def _create_slalom_left_pose(frame_id: str):
    """
    Create a PoseStamped for the left side of the slalom.
    """
    return create_stamped_pose(
        frame_id=frame_id,
        position_x=0.75,
        position_y=0.3,
        position_z=0.0,
        roll=-90.0,
        pitch=-90.0,
        yaw=0.0,  # Facing left
    )


def _create_slalom_right_pose(frame_id: str):
    """
    Create a PoseStamped for the right side of the slalom.
    """
    return create_stamped_pose(
        frame_id=frame_id,
        position_x=2.25,
        position_y=0.3,
        position_z=0.0,
        roll=-90.0,
        pitch=-90.0,
        yaw=0.0,
    )


def validate_clusters(
    layer_zero_to_one: TransformStamped,
    layer_one_to_two: TransformStamped,
) -> bool:
    """
    Validate that the clusters are not None and are spaced apart correctly.
    """

    def check_tf_dist(
        tf: TransformStamped,
        z_lower_bound: float = 1.0,
        z_upper_bound: float = 5.0,
    ) -> bool:
        """
        Check if the distance between two transforms is within a specified range in the z-axis.
        """
        return z_lower_bound < abs(tf.transform.translation.z) < z_upper_bound

    if layer_zero_to_one is None or layer_one_to_two is None:
        return False

    return check_tf_dist(layer_zero_to_one) and check_tf_dist(layer_one_to_two)


def create_move_to_layer_zero_root():
    """
    Create a root node that moves to the zero layer of the slalom channel.
    The movement strategy is as follows:
    A. Valid Clusters AND Invalid Clusters:
        1. Create a pose based on the clustered layer zero.
        2. Move to the clustered pose.
    We operate under the assumption that we always have the clustered pose for layer zero.
    """
    root = py_trees.composites.Sequence(
        name="Move to Layer Zero",
        memory=True,
    )

    set_layer_zero_clustered_pose = DynamicSetBlackboard(
        name="Set Layer Zero Clustered Pose",
        key=POSE_FUNC_KEY,
        update_key=LAYER_ZERO_POSE_KEY,
        overwrite=True,
        func=lambda f: f(LAYER_ZERO_CLUSTERED),
    )

    goto_layer_zero_clustered = goto.FromBlackboard(
        name="Goto Layer Zero Clustered",
        pose_key=LAYER_ZERO_POSE_KEY,
        depth_override_value=DEPTH_OVERRIDE_VALUE,
        specified_heading=False,
    )

    root.add_children(
        [
            set_layer_zero_clustered_pose,  # Set the pose for layer zero
            goto_layer_zero_clustered,  # Move to the clustered pose for layer zero
        ]
    )

    return root


def create_move_between_layers_root(
    current_layer: Literal[
        "zero",
        "one",
    ],
    next_layer: Literal[
        "one",
        "two",
    ],
):
    """
    Create a root node that moves between layers of the slalom channel.
    The movement strategy is as follows:
    A. Yaw or Sweep:
        A1. Yaw and Reclustering:
            0. Check that we have valid clusters.
            1. Get the transform from the current layer to the next layer.
            2. Create a yawed pose based on the transform.
            3. Move to the yawed pose.
            4. Recluster the next layer.
            5. Write the reclustered transform to the blackboard.
        A2. Sweep and Reclustering:
            0. Sweep left and right.
            1. Recluster the next layer.
            2. Write the reclustered transform to the blackboard.
    B. Reclustered or Hardcoded:
        B1. Valid Reclustered Pose:
            0. Write the validation of the reclustered layer to the blackboard.
            1. Check if the reclustering was valid.
            2. Set the next layer's pose to the reclustered pose.
        B2. Initial or Hardcoded Pose:
            0. Check that we have valid clusters.
            1. Set the next layer's pose to the initial pose.
            2. If we don't have valid clusters, set the next layer's pose to the hardcoded pose.
    C. Finally, move to the next layer's pose.
    """

    def validate_reclustered_layer(
        tf: TransformStamped,
    ):
        """
        The transform used here is from the current layer to the reclustered/next layer.
        We check if the transform is None or if the x distance is not within the expected range
        """

        def check_tf_dist(
            tf: TransformStamped,
            x_lower_bound: float = 1.0,
            x_upper_bound: float = 3.0,
        ) -> bool:
            """
            Check if the distance in the x-axis is within the specified bounds.
            """
            z_distance = abs(tf.transform.translation.x)
            return x_lower_bound < z_distance < x_upper_bound

        if tf is None:
            return False
        return check_tf_dist(tf)

    current_layer_frame = (
        LAYER_ZERO_CLUSTERED if current_layer == "zero" else LAYER_ONE_CLUSTERED
    )
    next_layer_frame = (
        LAYER_ONE_CLUSTERED if next_layer == "one" else LAYER_TWO_CLUSTERED
    )
    next_layer_hardcoded_frame = (
        LAYER_ONE_HARDCODED if next_layer == "one" else LAYER_TWO_HARDCODED
    )

    root = py_trees.composites.Sequence(
        name=f"Move from Layer {current_layer} to Layer {next_layer}",
        memory=True,
    )

    sel_yaw_or_sweep = py_trees.composites.Selector(
        name=f"Selector for Layer {current_layer} to Layer {next_layer}: Yaw or Sweep",
        memory=True,
    )

    seq_yaw_and_recluster = py_trees.composites.Sequence(
        name=f"Movement from Layer {current_layer} to Layer {next_layer} with Yaw and Reclustering",
        memory=True,
    )

    # Having valid clusters implies that the clusters are not None and are spaced apart correctly.
    check_is_valid_clusters = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check Valid Clusters",
        check=py_trees.common.ComparisonExpression(
            variable=IS_VALID_CLUSTERS_KEY,
            value=True,
            operator=operator.eq,
        ),
    )

    get_current_to_next_tf = create_tf_checker_from_constant_root(
        start_frames=[current_layer_frame],
        end_frames=[next_layer_frame],
        update_keys=[LAYER_TO_LAYER_TF_KEY],
        fallback_val=[None],
        is_fail_if_none=False,
    )

    create_yaw_view_pose = DynamicSetBlackboard(
        name=f"Create Yawed Pose for Layer {current_layer} to Layer {next_layer}",
        key=LAYER_TO_LAYER_TF_KEY,
        update_key=LAYER_TO_LAYER_POSE_KEY,
        overwrite=True,
        func=lambda tf: create_yawed_pose(tf=tf),
    )

    goto_yaw_view_pose = goto.FromBlackboard(
        name=f"Goto Yawed Pose from Layer {current_layer} to Layer {next_layer}",
        pose_key=LAYER_TO_LAYER_POSE_KEY,
        depth_override_value=DEPTH_OVERRIDE_VALUE,
    )

    recluster_action = shared_action_client.FromConstant(
        name=f"Recluster Transforms from Layer {current_layer} to Layer {next_layer}",
        shared_action=SharedAction.CLUSTER_MULTI,
        action_goal=create_clustering_goal(
            in_children=CLUSTERING_IN_CHILDREN_NEAR,
            out_children=[
                next_layer_frame + "/reclustered/yaw",
                LAYER_ONE_RECLUSTERED_DUMMY,
                LAYER_TWO_RECLUSTERED_DUMMY,
            ],
            duration=RECLUSTER_DURATION,
        ),
    )

    write_reclustered_to_bb = create_tf_checker_from_constant_root(
        start_frames=[BASE_LINK_FRAME],
        end_frames=[next_layer_frame + "/reclustered/yaw"],
        update_keys=[RECLUSTERED_LAYER_KEY],
        fallback_val=[None],
        is_fail_if_none=False,
    )

    set_next_layer_reclustered_pose_yaw = DynamicSetBlackboard(
        name=f"Set Next Layer {next_layer} Reclustered Pose",
        key=POSE_FUNC_KEY,
        update_key=LAYER_TO_LAYER_POSE_KEY,
        overwrite=True,
        func=lambda f: f(next_layer_frame + "/reclustered/yaw"),
    )

    seq_yaw_and_recluster.add_children(
        [
            check_is_valid_clusters,  # Check that we have valid clusters
            get_current_to_next_tf,  # Get the transform from current layer to next layer
            create_yaw_view_pose,  # Create a yawed pose based on the transform
            goto_yaw_view_pose,  # Move to the yawed pose
            recluster_action,  # Recluster the next layer
            write_reclustered_to_bb,  # Write the reclustered transform to the blackboard
            set_next_layer_reclustered_pose_yaw,
        ]
    )

    seq_sweep_and_recluster = py_trees.composites.Sequence(
        name=f"Movement from Layer {current_layer} to Layer {next_layer} with Sweep and Reclustering",
        memory=True,
    )

    sweep_poses = [
        create_stamped_pose(
            frame_id=BASE_LINK_FRAME, yaw=-SWEEP_ANGLE_DEGREES, use_radians=False
        ),
        create_stamped_pose(
            frame_id=BASE_LINK_FRAME, yaw=SWEEP_ANGLE_DEGREES * 2, use_radians=False
        ),
    ]

    goto_sweep_0 = goto.FromConstant(
        name=f"Sweep left from Layer {current_layer} to Layer {next_layer}",
        pose=sweep_poses[0],
        depth_override_value=DEPTH_OVERRIDE_VALUE,
    )

    goto_sweep_1 = goto.FromConstant(
        name=f"Sweep right from Layer {current_layer} to Layer {next_layer}",
        pose=sweep_poses[1],
        depth_override_value=DEPTH_OVERRIDE_VALUE,
    )

    recluster_sweep_pre = shared_action_client.FromConstant(
        name=f"Recluster before Sweep from Layer {current_layer} to Layer {next_layer}",
        shared_action=SharedAction.CLUSTER_MULTI,
        action_goal=create_clustering_goal(
            in_children=CLUSTERING_IN_CHILDREN_NEAR,
            out_children=[
                next_layer_frame + "/reclustered/sweep",
                LAYER_ONE_RECLUSTERED_DUMMY,
                LAYER_TWO_RECLUSTERED_DUMMY,
            ],
            duration=SWEEP_RECLUSTER_DURATION,
            persistent=True,
        ),
    )

    recluster_sweep_0 = shared_action_client.FromConstant(
        name=f"Recluster during Sweep from Layer {current_layer} to Layer {next_layer}",
        shared_action=SharedAction.CLUSTER_MULTI,
        action_goal=create_clustering_goal(
            in_children=CLUSTERING_IN_CHILDREN_NEAR,
            out_children=[
                next_layer_frame + "/reclustered/sweep",
                LAYER_ONE_RECLUSTERED_DUMMY,
                LAYER_TWO_RECLUSTERED_DUMMY,
            ],
            duration=SWEEP_RECLUSTER_DURATION,
            persistent=True,
        ),
    )

    recluster_sweep_1 = shared_action_client.FromConstant(
        name=f"Recluster during Sweep from Layer {current_layer} to Layer {next_layer}",
        shared_action=SharedAction.CLUSTER_MULTI,
        action_goal=create_clustering_goal(
            in_children=CLUSTERING_IN_CHILDREN_NEAR,
            out_children=[
                next_layer_frame + "/reclustered/sweep",
                LAYER_ONE_RECLUSTERED_DUMMY,
                LAYER_TWO_RECLUSTERED_DUMMY,
            ],
            duration=SWEEP_RECLUSTER_DURATION,
            persistent=True,
        ),
    )

    write_sweep_reclustered_to_bb = create_tf_checker_from_constant_root(
        start_frames=[BASE_LINK_FRAME],
        end_frames=[next_layer_frame + "/reclustered/sweep"],
        update_keys=[RECLUSTERED_LAYER_KEY],
        fallback_val=[None],
        is_fail_if_none=False,
    )

    set_next_layer_reclustered_pose_sweep = DynamicSetBlackboard(
        name=f"Set Next Layer {next_layer} Reclustered Pose",
        key=POSE_FUNC_KEY,
        update_key=LAYER_TO_LAYER_POSE_KEY,
        overwrite=True,
        func=lambda f: f(next_layer_frame + "/reclustered/sweep"),
    )

    seq_sweep_and_recluster.add_children(
        [
            recluster_sweep_pre,  # Recluster before sweeping
            goto_sweep_0,  # Sweep left
            recluster_sweep_0,  # Recluster during the left sweep
            goto_sweep_1,  # Sweep right
            recluster_sweep_1,  # Recluster during the right sweep
            write_sweep_reclustered_to_bb,  # Write the reclustered transform to the blackboard
            set_next_layer_reclustered_pose_sweep,
        ]
    )

    sel_yaw_or_sweep.add_children(
        [
            seq_yaw_and_recluster,
            seq_sweep_and_recluster,
        ]
    )

    sel_reclustered_or_hardcoded = py_trees.composites.Selector(
        name=f"Selector for Layer {current_layer} to Layer {next_layer}: Reclustered or Hardcoded",
        memory=True,
    )

    seq_valid_recluster = py_trees.composites.Sequence(
        name=f"Movement from Layer {current_layer} to Layer {next_layer} with Valid Reclustered Pose",
        memory=True,
    )

    write_recluster_layer_validation = DynamicSetBlackboard(
        name="Set Is Valid Re-Clusters After Recluster",
        key=RECLUSTERED_LAYER_KEY,
        update_key=IS_VALID_RECLUSTERED_KEY,
        overwrite=True,
        func=lambda tf: validate_reclustered_layer(tf),
    )

    check_reclustered_layer_validity = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check if Reclustering was Valid",
        check=py_trees.common.ComparisonExpression(
            variable=IS_VALID_RECLUSTERED_KEY,
            value=True,
            operator=operator.eq,
        ),
    )

    seq_valid_recluster.add_children(
        [
            write_recluster_layer_validation,  # Write the validation of the reclustered layer to the blackboard
            check_reclustered_layer_validity,  # Check if the reclustering was valid
        ]
    )

    sel_initial_or_hardcoded = py_trees.composites.Selector(
        name=f"Selector for Layer {current_layer} to Layer {next_layer}: Initial or Hardcoded",
        memory=True,
    )

    seq_initial = py_trees.composites.Sequence(
        name=f"Movement from Layer {current_layer} to Layer {next_layer} with Initials",
        memory=True,
    )

    check_initials = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check Valid Clusters",
        check=py_trees.common.ComparisonExpression(
            variable=IS_VALID_CLUSTERS_KEY,
            value=True,
            operator=operator.eq,
        ),
    )

    set_next_layer_initial_pose = DynamicSetBlackboard(
        name=f"Set Next Layer {next_layer} Initial Pose",
        key=POSE_FUNC_KEY,
        update_key=LAYER_TO_LAYER_POSE_KEY,
        overwrite=True,
        func=lambda f: f(next_layer_frame),
    )

    seq_initial.add_children(
        [
            check_initials,  # Check that we have valid clusters
            set_next_layer_initial_pose,  # Set the next layer's pose to the initial pose
        ]
    )

    seq_hardcoded = py_trees.composites.Sequence(
        name=f"Movement from Layer {current_layer} to Layer {next_layer} with Hardcoded Pose",
        memory=True,
    )

    set_hardcoded = DynamicSetBlackboard(
        name="Set Next Layer Hardcoded Pose",
        key=POSE_FUNC_KEY,
        update_key=LAYER_TO_LAYER_POSE_KEY,
        overwrite=True,
        func=lambda f: f(next_layer_hardcoded_frame),
    )

    seq_hardcoded.add_children(
        [
            set_hardcoded,  # Set the next layer's pose to the hardcoded pose
        ]
    )

    sel_initial_or_hardcoded.add_children(
        [
            seq_initial,  # If we have valid clusters, we can move to the next layer
            seq_hardcoded,  # If we don't have valid clusters, we can move to the hardcoded pose
        ]
    )

    sel_reclustered_or_hardcoded.add_children(
        [
            seq_valid_recluster,  # If we have valid reclustered pose, we can move to the next layer
            sel_initial_or_hardcoded,  # If we don't have valid reclustered pose, we can either move to the initial pose or the hardcoded pose
        ]
    )

    goto_next_layer = goto.FromBlackboard(
        name=f"Goto Next Layer {next_layer}",
        pose_key=LAYER_TO_LAYER_POSE_KEY,
        depth_override_value=DEPTH_OVERRIDE_VALUE,
        specified_heading=False,
    )

    root.add_children(
        [
            sel_yaw_or_sweep,  # Select between yawing or sweeping
            sel_reclustered_or_hardcoded,  # Select between reclustered or hardcoded pose
            goto_next_layer,  # Finally, move to the next layer's pose
        ]
    )

    return root


def create_movement_strategy_root():
    """
    Create a root node that implements the movement strategy for the slalom channel.
    The movement strategy is as follows:
    0. Set the pose function based on whether the slalom is left or right.
    1. Write the clustered transforms of the layers to the blackboard.
    2. Set the is_valid_clusters to True if the clusters are valid.
    3. Move to the zero layer of the slalom channel.
    4. Move to the one layer of the slalom channel.
    5. Move to the two layer of the slalom channel.
    """

    root = py_trees.composites.Sequence(
        name="Mixed Slalom Movement",
        memory=True,
    )

    set_pose_func = DynamicSetBlackboard(
        name="Set Pose Function",
        key=IS_LEFT_KEY,
        update_key=POSE_FUNC_KEY,
        overwrite=True,
        func=lambda is_left: (
            _create_slalom_left_pose if is_left else _create_slalom_right_pose
        ),
    )

    write_transforms_to_bb = create_tf_checker_from_constant_root(
        start_frames=[
            LAYER_ZERO_CLUSTERED,
            LAYER_ONE_CLUSTERED,
        ],
        end_frames=[
            LAYER_ONE_CLUSTERED,
            LAYER_TWO_CLUSTERED,
        ],
        update_keys=[
            LAYER_ZERO_TO_ONE_CLUSTERED_TF_KEY,
            LAYER_ONE_TO_TWO_CLUSTERED_TF_KEY,
        ],
        fallback_val=[
            None,
            None,
        ],
        is_fail_if_none=False,
    )

    # Having valid clusters implies that the clustered layers are not None and are spaced apart correctly.
    set_is_valid_clusters = DynamicSetBlackboard(
        name="Set Is Valid Clusters",
        key=[
            LAYER_ZERO_TO_ONE_CLUSTERED_TF_KEY,
            LAYER_ONE_TO_TWO_CLUSTERED_TF_KEY,
        ],
        update_key=IS_VALID_CLUSTERS_KEY,
        overwrite=True,
        func=lambda layer_zero_to_one, layer_one_to_two: validate_clusters(
            layer_zero_to_one,
            layer_one_to_two,
        ),
    )

    move_to_center = goto.FromConstant(
        name="Move to centre",
        pose=create_stamped_pose(
            frame_id="slalom/centre",
        ),
        depth_override_value=DEPTH_OVERRIDE_VALUE,
    )

    move_to_layer_zero = create_move_to_layer_zero_root()
    move_to_layer_one = create_move_between_layers_root(
        current_layer="zero",
        next_layer="one",
    )
    move_to_layer_two = create_move_between_layers_root(
        current_layer="one",
        next_layer="two",
    )

    root.add_children(
        [
            set_pose_func,
            write_transforms_to_bb,
            set_is_valid_clusters,
            move_to_center,
            move_to_layer_zero,
            move_to_layer_one,
            move_to_layer_two,
        ]
    )

    return root
