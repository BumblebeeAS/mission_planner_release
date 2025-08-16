import py_trees
from geometry_msgs.msg import TransformStamped

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import (
    create_stamped_pose,
)
from mission_planner_2.commons.tf_checker import create_tf_checker_from_constant_root
from mission_planner_2.trees.auv.goto import goto

# Generate namespace automatically from file path DONT set manually
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

_ODOM_TF_KEY = fk("odom_tf")
_GOTO_POSE_KEY = fk("goto_pose")


def create_move_to_task(
    task: str,
    start: dict,
    end: dict,
    zero_yaw_key: str,
    goto_depth: float = 0.3,
    specified_heading: bool = True,
):
    root = py_trees.composites.Sequence(
        name=f"Move to {task}",
        memory=True,
    )

    get_odom = create_tf_checker_from_constant_root(
        start_frames=["world_ned"],
        end_frames=["auv4/base_link_ned"],
        update_keys=[_ODOM_TF_KEY],
        fallback_val=[None],
    )

    dynamic_create_zero_yaw_pose = DynamicSetBlackboard(
        name="Set zero yaw pose",
        key=[
            _ODOM_TF_KEY,
            zero_yaw_key,
        ],
        update_key=_GOTO_POSE_KEY,
        overwrite=True,
        func=get_zero_yaw_pose,
    )

    goto_zero_yaw = goto.FromBlackboard(
        name="Goto zero yaw",
        pose_key=_GOTO_POSE_KEY,
        depth_override_value=goto_depth,
        stabilize_duration=8,
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
        depth_override_value=goto_depth,
        specified_heading=specified_heading,
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
    output_vector_as_dict["yaw"] = end["yaw"] - start["yaw"]

    return output_vector_as_dict


# def get_zero_yaw_pose(
#     tf: TransformStamped | None,
# ):
#     if tf is None:
#         return create_stamped_pose(
#             frame_id="auv4/base_link_ned",
#         )
#     return create_stamped_pose(
#         frame_id="world_ned",
#         yaw=0.0,
#         position_x=tf.transform.translation.x,
#         position_y=tf.transform.translation.y,
#         position_z=tf.transform.translation.z,
#         use_radians=True,
#     )


def get_zero_yaw_pose(
    odom_tf: TransformStamped | None,
    zero_yaw: float,
):
    if odom_tf is None:
        return create_stamped_pose(
            frame_id="auv4/base_link_ned",
        )

    return create_stamped_pose(
        frame_id="world_ned",
        yaw=zero_yaw,
        position_x=odom_tf.transform.translation.x,
        position_y=odom_tf.transform.translation.y,
        position_z=odom_tf.transform.translation.z,
        use_radians=True,
    )
