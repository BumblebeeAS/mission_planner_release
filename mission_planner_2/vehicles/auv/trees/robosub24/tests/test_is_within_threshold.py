import py_trees
from geometry_msgs.msg import PoseStamped

from mission_planner_2 import is_within_threshold

test_pose: PoseStamped = PoseStamped()
test_pose.header.frame_id = "world_ned"
test_pose.pose.position.x = 1.0
test_pose.pose.position.y = 0.0
test_pose.pose.position.z = 0.0
test_pose.pose.orientation.w = 1.0
test_pose.pose.orientation.x = 0.0
test_pose.pose.orientation.y = 0.0
test_pose.pose.orientation.z = 0.0

test_target_pose: PoseStamped = PoseStamped()
test_target_pose.header.frame_id = "world_ned"
test_target_pose.pose.position.x = 1.9
test_target_pose.pose.position.y = 0.0
test_target_pose.pose.position.z = 0.0
test_target_pose.pose.orientation.w = 1.0
test_target_pose.pose.orientation.x = 0.0
test_target_pose.pose.orientation.y = 0.0
test_target_pose.pose.orientation.z = 0.0

test_threshold = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]


def create_is_within_threshold_test() -> py_trees.common.Status:

    key_pose = "test_pose"
    key_target_pose = "test_target_pose"
    key_threshold = "test_threshold"

    root = py_trees.composites.Sequence(
        name="test_is_within_threshold",
        memory=True,
    )

    threshold_check_blackboard = is_within_threshold.FromBlackboard(
        name="threshold_check_blackboard",
        key_pose=key_pose,
        key_target_pose=key_target_pose,
        key_threshold=key_threshold,
    )

    threshold_check_constant = is_within_threshold.FromConstant(
        name="threshold_check_constant",
        key_pose=key_pose,
        key_target_pose=key_target_pose,
        threshold=test_threshold,
    )

    set_pose_in_bb = py_trees.behaviours.SetBlackboardVariable(
        name="set_pose",
        variable_name=key_pose,
        variable_value=test_pose,
        overwrite=True,
    )

    set_target_pose_in_bb = py_trees.behaviours.SetBlackboardVariable(
        name="set_target_pose",
        variable_name=key_target_pose,
        variable_value=test_target_pose,
        overwrite=True,
    )

    set_threshold_in_bb = py_trees.behaviours.SetBlackboardVariable(
        name="set_threshold",
        variable_name=key_threshold,
        variable_value=test_threshold,
        overwrite=True,
    )

    set_success = py_trees.behaviours.SetBlackboardVariable(
        name="succeed threshold",
        variable_name="stc",
        variable_value="succeeded",
        overwrite=True,
    )

    fallback = py_trees.composites.Selector(
        name="select if not threshold",
        memory=True,
        children=[threshold_check_blackboard, set_success],
    )

    root.add_children(
        [
            set_pose_in_bb,
            set_target_pose_in_bb,
            set_threshold_in_bb,
            fallback,
        ]
    )

    return root
