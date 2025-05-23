import operator

import py_trees
import py_trees_ros
from bb_msgs.srv import IMPoseEstimatorToggleTemplate
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import UInt8

from mission_planner_2.commons.blackboard import full_key_generator
from mission_planner_2.commons.namespace_utils import generate_namespace
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto_node
from mission_planner_2.trees.auv.bin.move_to_task import create_move_to_bin_task_root


NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)


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

    enable_detections = py_trees_ros.service_clients.FromConstant(
        name="Enable Detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(
            enabled=True,
            template_name="Task03_DropBRUVS.png",
            camera_frame_id="auv4/bot_cam_optical"
        ),
        key_response=fk("bin_enable_detections"),
    )

    disable_detections = py_trees_ros.service_clients.FromConstant(
        name="Disable Detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(enabled=False),
        key_response=fk("bin_disable_detections"),
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
            value=False,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    align_to_target = goto_node.FromBlackboard(
        name="Align to Target",
        parent_namespace=NAMESPACE,
        pose_key="bin_pose",
    )

    # For Yosie

    # subscriber --> yaw (NED)

    # rotation_pose = create_stamped_pose(
    #     "auv4/base_link_ned",
    #     0.0,  # Temporary, please update
    #     0.0,
    #     0.0,
    #     0.0,
    #     0.0,
    #     yaw,
    # )

    # goto to rotation pose

    # service call to /auv4/choice std_srvs::Trigger (py_trees API tells you how to do this)

    # choose between offsets based on choice (see how torpedo does it)

    # goto that offset

    bin_pose = create_stamped_pose(
        "Task03_DropBRUVS_optical/clustered",
        0.0, # Temporary, please update
        0.0,
        0.0,
        0.0,
        0.0,
        0.0
    )

    align_to_target_const = goto_node.FromConstant(
        name="Align to Target",
        parent_namespace=NAMESPACE,
        pose=bin_pose
    )

    set_dropper_actuation = py_trees.behaviours.SetBlackboardVariable(
        name="Set Drop Bin Actuation",
        variable_name=fk("bin_actuation"),
        variable_value=UInt8(data=6),
        overwrite=True,
    )

    fire_dropper_1 = py_trees_ros.publishers.FromBlackboard(
        name="Drop the BRUV 1",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=fk("bin_actuation"),
    )

    fire_dropper_2 = py_trees_ros.publishers.FromBlackboard(
        name="Drop the BRUV 2",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=fk("bin_actuation"),
    )

    launch_seq.add_children(
        children=[
            enable_detections,
            enable_detections_succeeded,
            py_trees.timers.Timer("wait for match", duration=10.0),
            align_to_target_const,
            py_trees.timers.Timer("wait to stabilize", duration=5.0),
            set_dropper_actuation,
            fire_dropper_1,
            py_trees.timers.Timer("delay between drops", duration=5.0),
            fire_dropper_2,
            disable_detections,
            disable_detections_succeeded,
        ],
    )

    root.add_children(
        children=[
            create_move_to_bin_task_root(),
            py_trees.timers.Timer("stabilise before match", duration=10.0),
            launch_seq,
        ]
    )

    return root
