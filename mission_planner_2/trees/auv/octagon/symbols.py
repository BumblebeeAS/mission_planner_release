import py_trees

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.tf_checker import create_tf_checker_from_constant_root
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.octagon.helpers import get_table_to_symbol_pose

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

_FISH_TF_KEY = fk("fish_tf")
_SHARK_TF_KEY = fk("shark_tf")
_LOOK_AT_TARGET_POSE_KEY = fk("look_at_target_pose")


def create_look_at_target_root(
    choice_key: str,
    fish_frame_clustered: str,
    shark_frame_clustered: str,
    table_center_frame_clustered: str,
    pause_duration: float,
):
    """
    Looks at the target.

    We use the table as a fixed point because the odometry xyz positions may drift over time.
    """
    root = py_trees.composites.Sequence(
        name="Look at target",
        memory=True,
    )

    tf_checker = create_tf_checker_from_constant_root(
        start_frames=[
            table_center_frame_clustered,
            table_center_frame_clustered,
        ],
        end_frames=[
            fish_frame_clustered,
            shark_frame_clustered,
        ],
        update_keys=[
            _FISH_TF_KEY,
            _SHARK_TF_KEY,
        ],
        fallback_val=[
            fish_frame_clustered,
            shark_frame_clustered,
        ],
    )

    dynamic_set_target_pose = DynamicSetBlackboard(
        name="Set target pose",
        key=[choice_key, _FISH_TF_KEY, _SHARK_TF_KEY],
        update_key=_LOOK_AT_TARGET_POSE_KEY,
        overwrite=True,
        func=get_table_to_symbol_pose,
    )

    goto_look_at_target_pose = goto.FromBlackboard(
        name="Goto look at target pose",
        pose_key=_LOOK_AT_TARGET_POSE_KEY,
        ignore_depth=True,
    )

    timer = py_trees.timers.Timer(
        name="Pause for scoring",
        duration=pause_duration,
    )

    root.add_children(
        children=[
            tf_checker,
            dynamic_set_target_pose,
            goto_look_at_target_pose,
            timer,
        ]
    )

    return root
