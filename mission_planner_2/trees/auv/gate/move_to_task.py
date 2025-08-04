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


def create_move_to_gate_task_root(
    world_coords: dict,
    relative_coords: dict,
    flipped_relative_coords: dict,
    is_relative: bool = True,
    is_flip: bool = False,
):
    root = py_trees.composites.Sequence(
        name="Move to gate task",
        memory=True,
    )

    if is_relative:
        frame = "auv4/base_link_ned"
        coords = flipped_relative_coords if is_flip else relative_coords
    else:
        frame = "map_ned"
        coords = world_coords

    gate_init_pose = create_stamped_pose(
        frame_id=frame,
        position_x=coords["x"],
        position_y=coords["y"],
        position_z=coords["z"],
        roll=coords["roll"],
        pitch=coords["pitch"],
        yaw=coords["yaw"],
    )

    move_to_gate = goto.FromConstant(
        "Move to gate",
        gate_init_pose,
    )

    root.add_children(children=[move_to_gate])

    return root
