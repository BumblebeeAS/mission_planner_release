import py_trees
import py_trees_ros

from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.miniauv.goto import goto
from mission_planner_2.trees.miniauv.utils.controls_init import (
    create_init_controls_root,
)

BASE_LINK_FRAME = "orca4_ned"
FIXED_DEPTH = 1.0


def create_goto_test():
    root = py_trees.composites.Sequence(name="Goto test", memory=True)

    seq_controls_init = create_init_controls_root(FIXED_DEPTH)

    test_pose = create_stamped_pose(frame_id=BASE_LINK_FRAME, position_x=5.0)

    goto_action = goto.FromConstant(name="Goto one pose", pose=test_pose)

    root.add_children([seq_controls_init, goto_action])

    return root
