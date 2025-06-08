import py_trees

from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto


def create_move_to_bin_task_root() -> py_trees.behaviour.Behaviour:
    """
    Create the root of the bin tree.
    """
    root = py_trees.composites.Sequence(
        name="Move to Bin Task Root",
        memory=True,
    )

    bin_init_pose = create_stamped_pose(
        "world_ned", position_x=0.0, position_y=0.0, position_z=0.8, yaw=0.0
    )

    move_to_bin = goto.FromConstant("move to bin", bin_init_pose)

    root.add_children(
        [
            move_to_bin,
        ]
    )

    return root
