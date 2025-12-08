import py_trees

from mission_planner_2.common.util.pose_utils import create_stamped_pose
from mission_planner_2.vehicles.auv.trees.robosub24.goto import goto


def create_goto_test():
    root = py_trees.composites.Sequence(
        name="test_goto",
        memory=True,
    )

    root.add_children(
        [
            goto.FromConstant(
                name="goto",
                pose=create_stamped_pose("auv4/base_link_ned", position_x=5.0),
            ),
        ]
    )

    return root
