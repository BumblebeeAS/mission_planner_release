import py_trees
from geometry_msgs.msg import PoseStamped

from mission_planner_2.trees.auv.goto import create_goto_root

test_pose: PoseStamped = PoseStamped()
test_pose.header.frame_id = "world_ned"
test_pose.pose.position.x = 1.0
test_pose.pose.position.y = 0.0
test_pose.pose.position.z = 0.0


def create_goto_test() -> py_trees.common.Status:
    root = py_trees.composites.Sequence(
        name="test_goto",
        memory=True,
    )

    goto = create_goto_root()

    set_pose_in_bb = py_trees.behaviours.SetBlackboardVariable(
        name="set_goto_pose",
        variable_name="input_pose_to_goto",
        variable_value=test_pose,
        overwrite=True,
    )

    root.add_children(
        [
            set_pose_in_bb,
            goto,
        ]
    )

    return root
