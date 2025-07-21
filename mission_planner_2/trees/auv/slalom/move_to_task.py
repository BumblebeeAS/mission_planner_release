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


def create_move_to_slalom_task_root(world_coords: dict):
    """
    Create the root of the slalom tree.
    """
    root = py_trees.composites.Sequence(
        name="Move to slalom task",
        memory=True,
    )

    """
    For sim.

    slalom_init_pose = create_stamped_pose(
        "world_ned", position_x=6.0, position_y=-0.6, position_z=0.7, yaw=-90.0
    )
    """
    slalom_init_pose = create_stamped_pose(
        "auv4/base_link_ned",
        position_x=world_coords["x"],
        position_y=world_coords["y"],
        position_z=world_coords["z"],
        roll=world_coords["roll"],
        pitch=world_coords["pitch"],
        yaw=world_coords["yaw"],
    )

    move_to_slalom = goto.FromConstant(
        "Move to slalom",
        slalom_init_pose,
    )

    root.add_children(children=[move_to_slalom])

    return root
