import py_trees
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from mission_planner_2.trees.auv.goto import (
    goto_with_threshold,
    goto_with_threshold_node,
)

test_pose: PoseStamped = PoseStamped()
test_pose.header.frame_id = "world_ned"
test_pose.pose.position.x = 1.0
test_pose.pose.position.y = 0.0
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


def create_goto_w_threshold_test() -> py_trees.common.Status:
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

    set_threshold = py_trees.behaviours.SetBlackboardVariable(
        name="set_threshold",
        variable_name="threshold_key",
        variable_value=[0.5, 0.1, 0.1, 0.0, 0.0, 0.0],
        overwrite=True,
    )

    root.add_children(
        [
            set_pose_in_bb,
            set_threshold,
            goto_with_threshold.create_goto_with_threshold_root(),
            # goto_with_threshold_node.FromBlackboard(
            #     name="goto_w_threshold",
            #     pose_key="input_pose_to_goto",
            #     topic_name="/auv4/nav/odom_ned",
            #     topic_type=Odometry,
            #     threshold_key="threshold",
            #     qos_profile=10,
            # ),
        ]
    )

    return root
