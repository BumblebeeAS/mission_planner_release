import py_trees

from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto

# Generate namespace automatically from file path
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)


def create_move_to_task_root():
    """
    Create the root of the slalom tree.
    """
    root = py_trees.composites.Sequence(
        name="Move to slalom task",
        memory=True,
    )

    # TO UPDATE: Set the initial pose for the slalom task
    slalom_init_pose = create_stamped_pose(
        "world_ned", position_x=0.0, position_y=0.0, position_z=0.8, yaw=0.0
    )

    """
    For sim.

    slalom_init_pose = create_stamped_pose(
        "world_ned", position_x=6.0, position_y=-0.6, position_z=0.7, yaw=-90.0
    )
    """
    slalom_init_pose = create_stamped_pose(
        "world_ned", position_x=6.0, position_y=-0.6, position_z=0.7, yaw=-90.0
    )

    move_to_slalom = goto.FromConstant(
        "Move to slalom",
        slalom_init_pose,
    )

    root.add_children(children=[move_to_slalom])

    return root
