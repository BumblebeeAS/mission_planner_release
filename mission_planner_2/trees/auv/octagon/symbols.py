import py_trees
from geometry_msgs.msg import TransformStamped
from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)


def get_look_at_target_pose(
    table_center_frame_clustered: str, table_to_surface_target_yaw: float
) -> TransformStamped:
    """Get the target pose to look at after trash pick up. Uses the previously saved
    table to target yaw to get the yawed pose in the table center frame."""
    target_pose = create_stamped_pose(
        frame_id=table_center_frame_clustered,
        yaw=table_to_surface_target_yaw,
        use_radians=True,
    )
    return target_pose


def create_look_at_target_root(
    table_center_frame_clustered: str,
    table_to_surface_target_yaw_key: str,
    look_at_target_pose_key: str,
    pause_duration: float,
):
    """Looks at the target.
    Uses the previously saved table to target yaw to get the yawed pose in the clustered table center frame.
    Goes to this yawed pose while ignoring depth.

    We use the table as a fixed point because the odometry xyz positions may drift over time.
    """
    root = py_trees.composites.Sequence(
        name=f"Look at target",
        memory=True,
    )
    dynamic_set_look_at_target_pose = DynamicSetBlackboard(
        name="Set look at target pose",
        key=[table_to_surface_target_yaw_key],
        update_key=look_at_target_pose_key,
        overwrite=True,
        func=lambda table_to_surface_target_yaw: get_look_at_target_pose(
            table_center_frame_clustered, table_to_surface_target_yaw
        ),
    )
    goto_look_at_target_pose = goto.FromBlackboard(
        name="Goto look at target pose",
        pose_key=look_at_target_pose_key,
        ignore_depth=True,
    )
    timer = py_trees.timers.Timer(
        name="Pause for scoring",
        duration=pause_duration,
    )
    root.add_children(
        children=[
            dynamic_set_look_at_target_pose,
            goto_look_at_target_pose,
            timer,
        ]
    )

    return root
