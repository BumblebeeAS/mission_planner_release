import operator

import py_trees
import py_trees.console as console
import py_trees_ros
import rclpy
from bb_msgs.srv import IMPoseEstimatorToggleTemplate
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time as rclpy_time

from mission_planner_2.trees.auv.goto import goto_node

NAMESPACE = "/auv4/gate_task"


def fk(key):
    """
    Generate the absolute blackboard key based on tree namespace and key name.
    """
    return py_trees.blackboard.Blackboard.absolute_name(NAMESPACE, key)


def _gen_enable_req():
    """
    Generate the enable service request to enable gate detections.
    """
    # TODO: This may not be the actual detection enable request
    req = IMPoseEstimatorToggleTemplate.Request()
    req.enabled = True
    req.template_name = "gate"
    return req


def _gen_disable_req():
    """
    Generate the disable service request to enable gate detections.
    """
    req = IMPoseEstimatorToggleTemplate.Request()
    req.enabled = False
    return req


def _gen_target_pose(frame):
    """
    Generate a PoseStamped message with all zeros for position and orientation,
    the current time, and the specified frame as the frame_id.

    Args:
        frame (str): The frame ID to use for the PoseStamped message

    Returns:
        PoseStamped: A PoseStamped message with the specified parameters
    """
    pose_stamped = PoseStamped()

    pose_stamped.header.frame_id = frame

    if rclpy.ok():
        node_time = rclpy_time()
        pose_stamped.header.stamp.sec = node_time.seconds_nanoseconds()[0]
        pose_stamped.header.stamp.nanosec = node_time.seconds_nanoseconds()[1]
    else:
        console.logwarn("rclpy has not init, using system time instead of ros time")
        import time

        current_time = time.time()
        pose_stamped.header.stamp.sec = int(current_time)
        pose_stamped.header.stamp.nanosec = int(
            (current_time - int(current_time)) * 1e9
        )

    pose_stamped.pose.position.x = 0.0
    pose_stamped.pose.position.y = 0.0
    pose_stamped.pose.position.z = 0.0

    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 1.0

    return pose_stamped


def _gen_init_pose():
    """
    Generate a PoseStamped message to move to initial position for gate task.
    """
    pose_stamped = PoseStamped()

    pose_stamped.header.frame_id = "world_ned"

    if rclpy.ok():
        node_time = rclpy_time()
        pose_stamped.header.stamp.sec = node_time.seconds_nanoseconds()[0]
        pose_stamped.header.stamp.nanosec = node_time.seconds_nanoseconds()[1]
    else:
        console.logwarn("rclpy has not init, using system time instead of ros time")
        import time

        current_time = time.time()
        pose_stamped.header.stamp.sec = int(current_time)
        pose_stamped.header.stamp.nanosec = int(
            (current_time - int(current_time)) * 1e9
        )

    pose_stamped.pose.position.x = 6.0
    pose_stamped.pose.position.y = 2.0
    pose_stamped.pose.position.z = 1.5

    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = -0.707
    pose_stamped.pose.orientation.w = 0.707

    return pose_stamped


def _gen_passthrough_pose():
    """
    Generate a PoseStamped message for passing through the gate.
    """
    pose_stamped = PoseStamped()

    pose_stamped.header.frame_id = "auv4/base_link"

    if rclpy.ok():
        node_time = rclpy_time()
        pose_stamped.header.stamp.sec = node_time.seconds_nanoseconds()[0]
        pose_stamped.header.stamp.nanosec = node_time.seconds_nanoseconds()[1]
    else:
        console.logwarn("rclpy has not init, using system time instead of ros time")
        import time

        current_time = time.time()
        pose_stamped.header.stamp.sec = int(current_time)
        pose_stamped.header.stamp.nanosec = int(
            (current_time - int(current_time)) * 1e9
        )

    pose_stamped.pose.position.x = 2.0
    pose_stamped.pose.position.y = 0.0
    pose_stamped.pose.position.z = 0.0

    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 1.0

    return pose_stamped


def create_gate_root():
    """
    Create the root of the gate tree.

    Execution Flow:
    1. Move closer to the gate
    2. Enable detections
    3. Check if enable succeeded
    4. Move to gate pose
    5. Pass through gate
    6. Disable detections
    7. Disable detections succeeded
    """

    TOGGLE_DETECTIONS_TOPIC = "/auv4/image_matching/toggle_template"

    root = py_trees.composites.Sequence(
        name="Gate Root",
        memory=True,
    )

    enable_detections = py_trees_ros.service_clients.FromConstant(
        name="Enable Detections",
        service_name=TOGGLE_DETECTIONS_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=_gen_enable_req(),
        key_response=fk("gate_enable_detections"),
    )

    disable_detections = py_trees_ros.service_clients.FromConstant(
        name="Disable Detections",
        service_name=TOGGLE_DETECTIONS_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=_gen_disable_req(),
        key_response=fk("gate_disable_detections"),
    )

    # check srv call succeeded from the BB
    enable_detections_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Enable Detections Succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("gate_enable_detections"),
            value=True,
            operator=lambda x, y: operator.eq(x, y),
        ),
    )

    disable_detections_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Disable Detections Succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("gate_disable_detections"),
            value=True,
            operator=lambda x, y: operator.eq(x, y),
        ),
    )

    gate_init_pose = _gen_init_pose()
    gate_target_pose = _gen_target_pose("auv4/gate")
    gate_passthrough_pose = _gen_passthrough_pose()

    move_towards_gate = goto_node.FromConstant(
        "move_towards_gate", NAMESPACE, gate_init_pose
    )

    move_to_gate_target = goto_node.FromConstant(
        "move_to_gate_target",
        NAMESPACE,
        gate_target_pose,
    )

    pass_through_gate = goto_node.FromConstant(
        "pass_through_gate", NAMESPACE, gate_passthrough_pose
    )

    root.add_children(
        children=[
            move_towards_gate,
            # enable_detections,
            # enable_detections_succeeded,
            move_to_gate_target,
            pass_through_gate,
            # disable_detections,
            # disable_detections_succeeded,
        ]
    )

    return root
