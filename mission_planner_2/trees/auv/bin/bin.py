import operator

import py_trees
import py_trees_ros
from bb_msgs.srv import IMPoseEstimatorToggleTemplate
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import UInt8

from mission_planner_2.commons import service_clients
from mission_planner_2.commons.blackboard import (
    DynamicSetBlackboard,
    full_key_generator
)
from mission_planner_2.commons.namespace_utils import generate_namespace
from mission_planner_2.trees.auv.goto import goto_node
from mission_planner_2.trees.auv.bin.move_to_task import create_move_to_task_root

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)


def _gen_enable_req():
    """
    Generate the service for the bin tree
    """
    req = IMPoseEstimatorToggleTemplate.Request()
    req.enabled = True
    req.template_name = "Task03_DropBRUVS.png"
    req.camera_frame_id = "auv4/bot_cam_optical" 
    return req

def _gen_disable_req():
    """
    Generate disable image matching service
    """
    req = IMPoseEstimatorToggleTemplate.Request()
    req.enabled = False
    return req

def _convert_pose(pose: PoseWithCovarianceStamped):
    pose_stamped = PoseStamped()
    pose_stamped.header = pose.header
    pose_stamped.pose = pose.pose.pose
    return pose_stamped

def create_bin_root():
    """
    Create the root of the bin tree.
    """
    TOGGLE_TEMPLATE_TOPIC = "/auv4/image_matching/toggle_template"

    root = py_trees.composites.Sequence(
        name="Bin Root",
        memory=True,
    )

    launch_seq = py_trees.composites.Sequence(
        name="Drop into Bin",
        memory=True,
    )

    # contains the logic for dropping BRUVS into the bin

    # 1 - move to task - in progress
    # 2 - enable detections + align to target
    # 3 - drop into bin twice
    # 4 - disable detections

    enable_detections = service_clients.FromConstant(
        name="Enable Detections",
        namespace=NAMESPACE,
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=_gen_enable_req(),
        key_response="bin_enable_detections",
    )

    disable_detections = service_clients.FromConstant(
        name="Disable Detections",
        namespace=NAMESPACE,
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=_gen_disable_req(),
        key_response="bin_disable_detections",
    )

    # check srv call succeeded from the BB
    enable_detections_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Enable Detections Succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("bin_enable_detections"),
            value=True,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    disable_detections_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Disable Detections Succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("bin_disable_detections"),
            value=True,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )
    
    pose_sub = py_trees_ros.subscribers.ToBlackboard(
        name="Pose Subscriber",
        topic_name="/auv4/bot_cam/image_matching/pose",
        topic_type=PoseWithCovarianceStamped,
        qos_profile=1,
        blackboard_variables={fk("bin_pose"): None},
    )

    convert_pose = DynamicSetBlackboard(
        name="Convert Pose",
        namespace=NAMESPACE,
        key="bin_pose",
        update_key="bin_pose",
        overwrite=True,
        func=_convert_pose,
    )

    align_to_target = goto_node.FromBlackboard(
        name="Align to Target",
        parent_namespace=NAMESPACE,
        pose_key="bin_pose",
    )

    set_torp_actuation = py_trees.behaviours.SetBlackboardVariable(
        name="Set Drop Bin Actuation",
        variable_name=fk("bin_actuation"),
        variable_value=6,
        overwrite=True,
    )

    fire_dropper_1 = py_trees_ros.publishers.FromBlackboard(
        name="Drop the BRUV 1",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=1,
        blackboard_variable=fk("bin_actuation"),
    )

    fire_dropper_2 = py_trees_ros.publishers.FromBlackboard(
        name="Drop the BRUV 2",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=1,
        blackboard_variable=fk("bin_actuation"),
    )

    launch_seq.add_children(
        children=[
            enable_detections,
            enable_detections_succeeded,
            py_trees.timers.Timer("wait for match", duration = 5),
            pose_sub,
            convert_pose,
            align_to_target,
            set_torp_actuation,
            fire_dropper_1,
            fire_dropper_2,
            disable_detections,
            disable_detections_succeeded,
        ],
    )

    root.add_children(
        children=[
            create_move_to_task_root(),
            py_trees.timers.Timer("stabilise before match", duration=5),
            launch_seq,
        ]
    )

    return root
