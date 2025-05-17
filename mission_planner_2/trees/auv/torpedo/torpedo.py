import operator

import py_trees
import py_trees_ros
from bb_msgs.srv import IMPoseEstimatorToggleTemplate
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import UInt8

from mission_planner_2.commons import service_clients
from mission_planner_2.commons.blackboard import (
    full_key_generator,
)
from mission_planner_2.commons.namespace_utils import generate_namespace
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.commons.sleep_node import SleepBehaviour
from mission_planner_2.trees.auv.goto import goto_node
from mission_planner_2.trees.auv.torpedo.move_to_task import create_move_to_task_root

# Generate namespace automatically from file path
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)


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

    enable_detections = service_clients.FromConstant(
        name="Enable Detections",
        namespace=NAMESPACE,
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=_gen_enable_req(),
        key_response="torpedo_enable_detections",
    )

    disable_detections = service_clients.FromConstant(
        name="Disable Detections",
        namespace=NAMESPACE,
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=_gen_disable_req(),
        key_response="torpedo_disable_detections",
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
            value=False,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    # UKF version
    # hole_pose = create_stamped_pose(
    #     "auv4/torpedo", 0.3, 0.0, 0.6, 90.0, 90.0, 0.0
    # )

    # Unfiltered/clustered version
    hole_pose = create_stamped_pose(
        "Task04_Tagging_01_optical", 0.3, 0.0, 0.6, 90.0, 90.0, 0.0
    )

    # For manual testing with dummy tfs (if you are too lazy to keep running image matching).
    # ros2 run tf2_ros static_transform_publisher -3.3 0 -0.9 0 0 1.57 world fake_det # usually the pose the detection gives
    # ros2 run tf2_ros static_transform_publisher 0.3 0 0.6 0 1.57 1.57 fake_det hole

    # align_pose = create_target_pose("hole")

    # TODO: if you don't do this, tf says you are trying to do a lookup into the future. should fix
    hole_pose.header.stamp = Time(sec=0, nanosec=0)

    align_to_target = goto_node.FromConstant(
        name="Align to Target",
        parent_namespace=NAMESPACE,
        pose=hole_pose,
        # anchor_frame_name="auv4/front_cam_optical",
    )

    # TODO: for now this is only one torpedo
    # later we will have to add the logic for more torpedoes

    set_torp_actuation = py_trees.behaviours.SetBlackboardVariable(
        name="Set Torpedo Actuation",
        variable_name=fk("torpedo_actuation"),
        variable_value=UInt8(data=2),
        overwrite=True,
    )

    fire_torpedo = py_trees_ros.publishers.FromBlackboard(
        name="Fire Torpedo",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=fk("torpedo_actuation"),
    )

    launch_seq.add_children(
        children=[
            enable_detections,
            enable_detections_succeeded,
            SleepBehaviour("wait for match", duration=2),
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
            SleepBehaviour("stabilise before match", duration=2),
            launch_seq,
        ]
    )

    return root
