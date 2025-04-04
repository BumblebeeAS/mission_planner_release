import py_trees
from geometry_msgs.msg import PoseStamped

from mission_planner_2.trees.auv.goto import goto_node
from mission_planner_2.trees.auv.goto.goto import create_goto_root

test_pose: PoseStamped = PoseStamped()
test_pose.header.frame_id = "world_ned"
test_pose.pose.position.x = 0.0
test_pose.pose.position.y = 1.0
test_pose.pose.position.z = 0.0

test_pose2: PoseStamped = PoseStamped()
test_pose2.header.frame_id = "world_ned"
test_pose2.pose.position.x = 1.0
test_pose2.pose.position.y = 0.0
test_pose2.pose.position.z = 0.5

test_pose3: PoseStamped = PoseStamped()
test_pose3.header.frame_id = "world_ned"
test_pose3.pose.position.x = 0.0
test_pose3.pose.position.y = 1.0
test_pose3.pose.position.z = 0.5


def create_goto_test() -> py_trees.common.Status:
    root = py_trees.composites.Sequence(
        name="test_goto",
        memory=True,
    )

    set_pose_in_bb = py_trees.behaviours.SetBlackboardVariable(
        name="set_goto_1",
        variable_name="input_pose_to_goto",
        variable_value=test_pose,
        overwrite=True,
    )

    set_pose_in_bb_2 = py_trees.behaviours.SetBlackboardVariable(
        name="set_goto_2",
        variable_name="input_pose_to_goto",
        variable_value=test_pose2,
        overwrite=True,
    )
    set_pose_in_bb_3 = py_trees.behaviours.SetBlackboardVariable(
        name="set_goto_3",
        variable_name="input_pose_to_goto",
        variable_value=test_pose3,
        overwrite=True,
    )

    root.add_children(
        [
            set_pose_in_bb,
            goto_node.FromBlackboard(
                name="goto",
                pose_key="input_pose_to_goto",
            ),
            # set_pose_in_bb_2,
            # goto_mine.FromBlackboard(name="goto2", pose_key="input_pose_to_goto"),
            # set_pose_in_bb_3,
            # goto_mine.FromBlackboard(name="goto3", pose_key="input_pose_to_goto"),
        ]
    )

    return root
