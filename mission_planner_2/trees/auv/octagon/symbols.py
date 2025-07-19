import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from geometry_msgs.msg import TransformStamped
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
    trash_name: str,
    table_center_frame: str,
    table_center_frame_clustered: str,
    table_to_surface_target_yaw_key: str,
    look_at_target_pose_key: str,
    cluster_duration: int,
):
    """Looks at the target after trash pick up.
    Clusters the table center, then uses the previously saved table to target yaw to
    get the yawed pose in the table center frame. Goes to this yawed pose while ignoring depth.

    We use the table as a fixed point because the odometry xyz positions may drift over time.
    """
    root = py_trees.composites.Sequence(
        name=f"Look at target ({trash_name})",
        memory=True,
    )
    cluster_table_centre = py_trees_ros.action_clients.FromConstant(
        name="Cluster table centre",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=table_center_frame,
            out_children=table_center_frame_clustered,
            duration=cluster_duration,
        ),
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
    # NOTE: For debugging
    timer = py_trees.timers.Timer(
        name="Wait for table cluster",
        duration=cluster_duration,
    )
    root.add_children(
        children=[
            cluster_table_centre,
            dynamic_set_look_at_target_pose,
            goto_look_at_target_pose,
            timer,
        ]
    )

    return root
