import operator

import py_trees
import py_trees_ros
from bb_msgs.srv import IMPoseEstimatorToggleTemplate
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import UInt8
import std_srvs

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.bins.move_to_task import create_move_to_bin_task_root


NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

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
    
     # Figure out the offset

TEMPLATE_3_OFFSET_SHARK = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}

TEMPLATE_3_OFFSET_FISH = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}

def _make_selection(choice: bool):
    """
    Function to set the offset based on choice.
    """
    if choice == True:
        offset = TEMPLATE_3_OFFSET_SHARK
    else:  # fish
        offset = TEMPLATE_3_OFFSET_FISH

    return create_stamped_pose(
        "Task03_DropBRUVS_optical/clustered",
        position_x=offset["x"],
        position_y=offset["y"],
        position_z=offset["z"],
        roll=offset["roll"],
        pitch=offset["pitch"],
        yaw=offset["yaw"],
    )

def create_bin_root():
    """
    Create the root of the bin tree.
    """
    TOGGLE_TEMPLATE_TOPIC = "/auv4/bot_cam/image_matching/toggle_template"

    root = py_trees.composites.Sequence(
        name="Bin Root",
        memory=True,
    )

    launch_seq = py_trees.composites.Sequence(
        name="Drop into Bin",
        memory=True,
    )

    # contains the logic for dropping BRUVS into the bin

    # 1 - move to task
    # 2 - enable detections 
    # 3 - service call to obtain choice
    # 4 - align to target
    # 5 - drop into bin twice
    # 6 - disable detections
    
    bin_pose = create_stamped_pose(
        "advay_please_remove_this",
        0.0, 
        0.0, 
        0.0,
        0.0,
        0.0,
        0.0,
    )

    go_to_bin = goto.FromConstant(
        "Go to bin",
        NAMESPACE,
        bin_pose
    )

    enable_detections = py_trees_ros.service_clients.FromConstant(
        name="Enable Detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(
            enabled=True,
            template_name="Task03_DropBRUVS.png",
            camera_frame_id="auv4/bot_cam_optical",
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

    choose_fish = py_trees_ros.service_clients.FromConstant(
        name="Get Choice",
        service_name="/auv4/choice/get_is_fish",
        service_type=std_srvs.srv.Trigger,
        service_request=std_srvs.srv.Trigger.Request(),
        key_response=fk("choice"),
    )

    set_choice = DynamicSetBlackboard(
        name="Set Choice",
        key="choice",
        namespace=NAMESPACE,
        update_key="position",
        overwrite=True,
        func=_make_selection,
    )

    align_to_target = goto.FromBlackboard(
        name="Align to Target",
        parent_namespace=NAMESPACE,
        pose_key="position",
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
            go_to_bin,
            enable_detections,
            enable_detections_succeeded,
            py_trees.timers.Timer("wait for match", duration=10.0),
            choose_fish,
            set_choice,
            align_to_target,
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
