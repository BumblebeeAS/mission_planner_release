from typing import Literal

import py_trees
from geometry_msgs.msg import TransformStamped
from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.trees.auv.goto import goto

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

########################## UPDATE CONSTANTS HERE #########################
# TODO: i think this was for non waypoint ver if dn can remove
WAIT_BETWEEN_MOVES = 10.0
TRANSFORM_CHECK_TIMEOUT = 5.0
#########################################################################

_MISSING_LAYER_KEY = fk("missing_layer")  # key for missing layer
_POSE_LIST_KEY = fk("pose_list")


def create_channel_movement_zero_root(
    slalom_frame_zero_clustered: str = "slalom_layer_0/clustered",
    slalom_frame_one_clustered: str = "slalom_layer_1/clustered",
    slalom_frame_two_clustered: str = "slalom_layer_2/clustered",
    create_func_key: str = "create_pose_func",
):
    root = py_trees.composites.Sequence(
        name="Channel Movement 0 missing",
        memory=True,
    )

    dynamic_create_pose_list = DynamicSetBlackboard(
        name="Dynamic Create Pose List 0 missing",
        key=create_func_key,
        update_key=_POSE_LIST_KEY,
        overwrite=True,
        func=lambda f: [
            f(slalom_frame_zero_clustered),
            f(slalom_frame_one_clustered),
            f(slalom_frame_two_clustered),
        ],
    )

    goto_zero_missing = goto.FromBlackboard(
        name="Goto Channel Movement 0 Missing",
        pose_key=_POSE_LIST_KEY,
        specified_heading=False,
    )

    root.add_children(
        [
            dynamic_create_pose_list,
            goto_zero_missing,
        ]
    )

    return root


def create_channel_movement_one_root(
    slalom_frame_zero_clustered: str = "slalom_layer_0/clustered",
    slalom_frame_one_clustered: str = "slalom_layer_1/clustered",
    slalom_frame_zero_key: str = "channel_pair_zero_tf",
    slalom_frame_one_key: str = "channel_pair_one_tf",
    slalom_one_from_zero_hardcoded: str = "slalom_layer_1/hardcoded",
    slalom_two_from_one_hardcoded: str = "slalom_layer_2/hardcoded",
    create_func_key: str = "create_pose_func",
):
    def check_tf_dist(
        tf_one: TransformStamped,
        tf_two: TransformStamped,
        dist_threshold: float = 3.0,
    ) -> Literal["one", "two"]:
        delta_z = abs(tf_one.transform.translation.z - tf_two.transform.translation.z)

        return "two" if delta_z < dist_threshold else "one"

    root = py_trees.composites.Sequence(
        name="Channel Movement 1 missing",
        memory=True,
    )

    dynamic_tf_check = DynamicSetBlackboard(
        name="Dynamic Transform Check missing layer",
        key=[slalom_frame_zero_key, slalom_frame_one_key],
        update_key=_MISSING_LAYER_KEY,
        overwrite=True,
        func=lambda x, y: check_tf_dist(x, y, dist_threshold=3.0),
    )

    sel_correct_missing_layer = py_trees.composites.Selector(
        name="Selector for missing layer",
        memory=True,
    )

    seq_missing_layer_one = py_trees.composites.Sequence(
        name="Seq missing layer one",
        memory=True,
    )

    missing_layer_one_check = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check is missing layer one",
        check=py_trees.common.ComparisonExpression(
            variable=_MISSING_LAYER_KEY,
            value="one",
            operator=lambda x, y: x == y,
        ),
    )

    dynamic_create_pose_list_one = DynamicSetBlackboard(
        name="Dynamic Create Pose List 1 missing (one)",
        key=create_func_key,
        update_key=_POSE_LIST_KEY,
        overwrite=True,
        func=lambda f: [
            f(slalom_frame_zero_clustered),
            f(slalom_one_from_zero_hardcoded),
            f(slalom_frame_one_clustered),
        ],
    )

    dynamic_create_pose_list_two = DynamicSetBlackboard(
        name="Dynamic Create Pose List 1 missing (two)",
        key=_POSE_LIST_KEY,
        update_key=create_func_key,
        overwrite=True,
        func=lambda f: [
            f(slalom_frame_zero_clustered),
            f(slalom_frame_one_clustered),
            f(slalom_two_from_one_hardcoded),
        ],
    )

    goto_one_missing = goto.FromBlackboard(
        name="Goto Channel Movement 1 Missing",
        pose_key=_POSE_LIST_KEY,
        specified_heading=False,
    )

    seq_missing_layer_one.add_children(
        [
            missing_layer_one_check,
            dynamic_create_pose_list_one,
        ]
    )

    sel_correct_missing_layer.add_children(
        [
            seq_missing_layer_one,
            dynamic_create_pose_list_two,
        ]
    )

    root.add_children(
        [
            dynamic_tf_check,
            sel_correct_missing_layer,
            goto_one_missing,
        ]
    )

    return root


def create_channel_movement_two_root(
    slalom_frame_zero_clustered: str = "slalom_layer_0/clustered",
    slalom_one_from_zero_hardcoded: str = "slalom_layer_1/hardcoded",
    slalom_two_from_one_hardcoded: str = "slalom_layer_2/hardcoded",
    create_func_key: str = "create_pose_func",
):
    root = py_trees.composites.Sequence(
        name="Channel Movement 2 missing",
        memory=True,
    )

    dynamic_create_pose_list = DynamicSetBlackboard(
        name="Dynamic Create Pose List 2 missing",
        key=create_func_key,
        update_key=_POSE_LIST_KEY,
        overwrite=True,
        func=lambda f: [
            f(slalom_frame_zero_clustered),
            f(slalom_one_from_zero_hardcoded),
            f(slalom_two_from_one_hardcoded),
        ],
    )

    goto_two_missing = goto.FromBlackboard(
        name="Goto Channel Movement 2 missing",
        pose_key=_POSE_LIST_KEY,
        specified_heading=False,
    )

    root.add_children(
        [
            dynamic_create_pose_list,
            goto_two_missing,
        ]
    )

    return root
