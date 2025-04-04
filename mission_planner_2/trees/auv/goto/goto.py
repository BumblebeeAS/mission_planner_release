import operator

import py_trees
import py_trees_ros
from bb_controls_msgs.action import Locomotion
from bb_planner_msgs.srv import GetPoseToControlsFrame
from numpy import rad2deg
from transforms3d.euler import quat2euler

from mission_planner_2 import dynamic_set_blackboard, service_clients


def _gen_goal(response: GetPoseToControlsFrame.Response):
    output_pose = response.output_pose.pose

    goal_msg = Locomotion.Goal()

    # Set the required fields
    goal_msg.move_rel = False
    goal_msg.depth_rel = False
    goal_msg.heading_rel = False
    try:
        goal_msg.depth_ctrl = Locomotion.Goal.DEPTH_MODE_DEPTH
    except Exception as e:
        print(e)
        goal_msg.depth_ctrl = 0

    goal_msg.specified_heading = True

    _, _, yaw = rad2deg(
        quat2euler(
            [
                output_pose.orientation.w,
                output_pose.orientation.x,
                output_pose.orientation.y,
                output_pose.orientation.z,
            ]
        )
    )

    # Set the setpoints
    goal_msg.forward_setpoints = [output_pose.position.x]
    goal_msg.sidemove_setpoints = [output_pose.position.y]
    goal_msg.depth_setpoints = [output_pose.position.z]
    goal_msg.heading_setpoints = [yaw]

    # Lists that need to be populated but aren't used
    goal_msg.roll_setpoints = [0.0]
    goal_msg.pitch_setpoints = [0.0]
    goal_msg.altitude_setpoints = []

    return goal_msg


def _gen_service_request(pose):
    req = GetPoseToControlsFrame.Request()
    req.input_pose = pose
    return req


def create_goto_root():
    """Creates the goto sequence to move to a predefined constant"""
    input_pose_to_goto = "input_pose_to_goto"
    controls_coverted_pose = "controls_converted_pose"
    action_goal = "action_goal"

    root = py_trees.composites.Sequence(
        name="GoTo",
        memory=True,
    )

    verify_bb_entry = py_trees.behaviours.CheckBlackboardVariableExists(
        name="check_waypoint_exists",
        variable_name=input_pose_to_goto,
    )

    create_srv_request = dynamic_set_blackboard.DynamicSetBlackboard(
        name="create_srv_request",
        key=input_pose_to_goto,
        update_key=input_pose_to_goto,
        func=_gen_service_request,
    )

    convert_frame_client = service_clients.FromBlackboard(
        name="convert_frame_client",
        service_type=GetPoseToControlsFrame,
        service_name="/auv4/convert_to_controls_pose",
        key_request=input_pose_to_goto,
        key_response=controls_coverted_pose,
    )

    check_srv_success = py_trees.behaviours.CheckBlackboardVariableValue(
        name="check_srv_success",
        check=py_trees.common.ComparisonExpression(
            variable=f"{controls_coverted_pose}.tf_success",
            operator=operator.eq,
            value=True,
        ),
    )

    convert_to_action_goal = dynamic_set_blackboard.DynamicSetBlackboard(
        name="convert_to_action_goal",
        key=controls_coverted_pose,
        update_key=action_goal,
        func=_gen_goal,
        overwrite=True,
    )

    send_action_goal = py_trees_ros.action_clients.FromBlackboard(
        name="send_action_goal",
        action_type=Locomotion,
        action_name="/auv4/controls",
        key=action_goal,
    )

    root.add_children(
        [
            verify_bb_entry,
            create_srv_request,
            convert_frame_client,
            check_srv_success,
            convert_to_action_goal,
            send_action_goal,
        ]
    )

    return root
