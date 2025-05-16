import operator

import py_trees
import py_trees_ros
from bb_msgs.srv import IMPoseEstimatorToggleTemplate
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import UInt8

from mission_planner_2.commons.blackboard import DynamicSetBlackboard, full_key
from mission_planner_2.trees.auv.goto import goto_node
from mission_planner_2.trees.auv.torpedo.move_to_task import create_move_to_task_root

NAMESPACE = "/auv4/torpedo_task"


def fk(key):
    """
    Generate the absolute blackboard key based on tree namespace and key name.
    """
    return full_key(NAMESPACE, key)


def _gen_enable_req():
    """
    Generate the service for the torpedo tree.
    """
    req = IMPoseEstimatorToggleTemplate.Request()
    req.enabled = True
    req.template_name = "Task04_Tagging_01.png"
    return req


def _gen_disable_req():
    """
    Generate the service for the torpedo tree.
    """
    req = IMPoseEstimatorToggleTemplate.Request()
    req.enabled = False
    return req


def _convert_pose(pose: PoseWithCovarianceStamped):
    """
    Convert the PoseWithCovarianceStamped message to a PoseStamped message.
    Adds the offset to the holes too.

    TODO: figure out the offset calculation laze for now.
    """
    pose_stamped = PoseStamped()
    pose_stamped.header = pose.header
    pose_stamped.pose = pose.pose.pose
    return pose_stamped


def create_torpedo_root():
    """
    Create the root of the torpedo tree.
    """
    TOGGLE_TEMPLATE_TOPIC = "/auv4/image_matching/toggle_template"

    root = py_trees.composites.Sequence(
        name="Torpedo Root",
        memory=True,
    )

    launch_seq = py_trees.composites.Sequence(
        name="Launch Torpedo",
        memory=True,
    )

    # contains the logic for launching the torpedo

    # 1 - move to task - in progress
    # 2 - enable detections + align to target
    # 3 - launch torpedo
    # 4 - disable detections

    enable_detections = py_trees_ros.service_clients.FromConstant(
        name="Enable Detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=_gen_enable_req(),
        key_response=fk("torpedo_enable_detections"),
    )

    disable_detections = py_trees_ros.service_clients.FromConstant(
        name="Disable Detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=_gen_disable_req(),
        key_response=fk("torpedo_disable_detections"),
    )

    # check srv call succeeded from the BB
    enable_detections_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Enable Detections Succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("torpedo_enable_detections"),
            value=True,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    disable_detections_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Disable Detections Succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("torpedo_disable_detections"),
            value=True,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    pose_sub = py_trees_ros.subscribers.ToBlackboard(
        name="Pose Subscriber",
        topic_name="/auv4/front_cam/image_matching/pose",
        topic_type=PoseWithCovarianceStamped,
        qos_profile=1,
        blackboard_variables={fk("torpedo_pose"): None},
    )

    convert_pose = DynamicSetBlackboard(
        name="Convert Pose",
        namespace=NAMESPACE,
        key="torpedo_pose",
        update_key="torpedo_pose",
        overwrite=True,
        func=_convert_pose,
    )

    align_to_target = goto_node.FromBlackboard(
        name="Align to Target",
        namespace=NAMESPACE,
        pose_key="torpedo_pose",
    )

    # TODO: for now this is only one torpedo
    # later we will have to add the logic for more torpedoes

    set_torp_actuation = py_trees.behaviours.SetBlackboardVariable(
        name="Set Torpedo Actuation",
        variable_name=fk("torpedo_actuation"),
        variable_value=2,
        overwrite=True,
    )

    fire_torpedo = py_trees_ros.publishers.FromBlackboard(
        name="Fire Torpedo",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=1,
        blackboard_variable=fk("torpedo_actuation"),
    )

    launch_seq.add_children(
        children=[
            enable_detections,
            enable_detections_succeeded,
            pose_sub,
            convert_pose,
            align_to_target,
            set_torp_actuation,
            fire_torpedo,
            disable_detections,
            disable_detections_succeeded,
        ],
    )

    root.add_children(
        children=[
            create_move_to_task_root(),
            launch_seq,
        ]
    )

    return root
