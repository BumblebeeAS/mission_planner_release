import py_trees
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.pose_utils import (
    create_stamped_pose,
)
from mission_planner_2.commons.tf_checker import create_tf_checker_from_constant_root
from mission_planner_2.trees.auv.goto import goto


def create_move_to_task(
    task: str, start: dict, end: dict, odom_key: str, zero_yaw_pose_key: str
):
    root = py_trees.composites.Sequence(
        name=f"Move to {task}",
        memory=True,
    )

    get_odom = create_tf_checker_from_constant_root(
        start_frames=["world_ned"],
        end_frames=["auv4/base_link_ned"],
        update_keys=[odom_key],
        fallback_val=[None],
    )

    dynamic_create_zero_yaw_pose = DynamicSetBlackboard(
        name="Set zero yaw pose",
        key=odom_key,
        update_key=zero_yaw_pose_key,
        overwrite=True,
        func=get_zero_yaw_pose,
    )

    goto_zero_yaw = goto.FromBlackboard(
        name="Goto zero yaw",
        pose_key=zero_yaw_pose_key,
        ignore_depth=True,
    )

    coords = compute_start_to_end_vector(start, end)

    task_pose = create_stamped_pose(
        "auv4/base_link_ned",
        position_x=coords["x"],
        position_y=coords["y"],
        position_z=coords["z"],
        roll=coords["roll"],
        pitch=coords["pitch"],
        yaw=coords["yaw"],
    )

    goto_task = goto.FromConstant(
        name=f"Goto {task} start",
        pose=task_pose,
        ignore_depth=True,
    )

    root.add_children(
        [
            get_odom,
            dynamic_create_zero_yaw_pose,
            goto_zero_yaw,
            goto_task,
        ]
    )

    return root


def compute_start_to_end_vector(start: dict, end: dict) -> dict:
    output_vector_as_dict = {}
    output_vector_as_dict["x"] = end["x"] - start["x"]
    output_vector_as_dict["y"] = end["y"] - start["y"]
    output_vector_as_dict["z"] = end["z"] - start["z"]
    output_vector_as_dict["roll"] = 0.0
    output_vector_as_dict["pitch"] = 0.0
    output_vector_as_dict["yaw"] = 0.0

    return output_vector_as_dict


def get_zero_yaw_pose(tf: TransformStamped):
    _, _, y = euler_from_quaternion(
        [
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ]
    )

    return create_stamped_pose(
        frame_id="auv4/base_link_ned",
        yaw=-y,
        use_radians=True,
    )
