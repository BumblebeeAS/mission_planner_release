import operator

import py_trees
import py_trees_ros
from bb_msgs.srv import IMPoseEstimatorToggleTemplate
from bb_perception_msgs.msg import PointCorrespondencesStamped
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
from mission_planner_2.trees.auv.bins.bin_selector import create_bin_selector_root

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

######################### UPDATE CONSTANTS HERE #########################
# Note that vision related nodes for bottom camera are not yet implemented
TOGGLE_TEMPLATE_TOPIC = "/auv4/bot_cam/image_matching/toggle_template"
POINT_CORRESPONDENCES_TOPIC = "/auv4/bot_cam/image_matching/point_correspondences"
TEMPLATE_NAME = "Task03_DropBRUVS.png"
CHOICE_KEY = "choice"
POSE_KEY = "pose"

# To update the transforms when we figure them out in cfg.yaml
FISH_BIN_FRAME = "auv4/bin/fish"
SHARK_BIN_FRAME = "auv4/bin/shark"
#########################################################################
def create_bin_root():
    """
    Create the root of the bin tree.
    """

    root = py_trees.composites.Sequence(
        name="Bin Root",
        memory=True,
    )

    launch_seq = py_trees.composites.Sequence(
        name="Drop into Bin",
        memory=True,
    )

    rotate_correctly = py_trees.composites.Sequence(
        name="Subsequence for rotating correctly",
        memory=True,
    )

    # contains the logic for dropping BRUVS into the bin
    # 1 - move to task
    # 2 - enable detections 
    # 3 - check for the correct orientation and go to correct orientation
    # 4 - service call to obtain choice and update desired pose based on choice
    # 5 - align to desired pose
    # 6 - drop into bin once
    # 7 - move slightly
    # 8 - drop into bin again
    # 9 - disable detections

    goto_centre = goto.FromConstant(
        name = "Goto the center of the bin",
        parent_namespace=NAMESPACE,
        pose=create_stamped_pose('auv4/bin/centre')
    )

    srv_enable_detections = py_trees_ros.service_clients.FromConstant(
        name="Enable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(
            enabled=True,
            camera_frame_id="auv4/bot_cam_optical",
            template_name=TEMPLATE_NAME,
        ),
        key_response=fk("bin_enable_detections"),
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

    get_points_1 = py_trees_ros.subscribers.ToBlackboard(
        name="get_points_1",
        topic_name=POINT_CORRESPONDENCES_TOPIC,
        topic_type=PointCorrespondencesStamped,
        qos_profile=qos_profile_system_default,
        blackboard_variables={fk("points_1"): "object_points"},
    )

    rotate_position = goto.FromConstant(
        name="rotate_position",
        parent_namespace=NAMESPACE,
        pose=create_stamped_pose("auv4/base_link_ned", yaw=180.0),
    )

    get_points_2 = py_trees_ros.subscribers.ToBlackboard(
        name="get_points_2",
        topic_name=POINT_CORRESPONDENCES_TOPIC,
        topic_type=PointCorrespondencesStamped,
        qos_profile=qos_profile_system_default,
        blackboard_variables={fk("points_2"): "object_points"},
    )

    compare_positions = DynamicSetBlackboard(
        name="compare_positions",
        namespace=NAMESPACE,
        key=["points_1", "points_2"],
        update_key="final",
        func=lambda x, y: create_stamped_pose("auv4/base_link_ned", yaw=180.0)
        if len(x.data) > len(y.data)
        else create_stamped_pose("auv4/base_link_ned"),
    )

    moveto_correct_position = goto.FromBlackboard(
        name="moveto_correct_position",
        parent_namespace=NAMESPACE,
        pose_key="final",
    )

    choose_fish = py_trees_ros.service_clients.FromConstant(
        name="Get Choice",
        service_name="/auv4/choice/get_is_fish",
        service_type=std_srvs.srv.Trigger,
        service_request=std_srvs.srv.Trigger.Request(),
        key_response=fk(CHOICE_KEY),
    )

    # Populate the desired pose based on choice
    update_selection = create_bin_selector_root(
        choice_key=fk(CHOICE_KEY),
        pose_key=fk(POSE_KEY)
    )

    align_to_target = goto.FromBlackboard(
        name="Align to Target",
        parent_namespace=NAMESPACE,
        pose_key=POSE_KEY,
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
    
    move_slightly = goto.FromConstant(
        name="move_slightly",
        parent_namespace=NAMESPACE,
        pose=create_stamped_pose("auv4/base_link_ned", x=0.05),
    )

    fire_dropper_2 = py_trees_ros.publishers.FromBlackboard(
        name="Drop the BRUV 2",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=fk("bin_actuation"),
    )

    disable_detections = py_trees_ros.service_clients.FromConstant(
        name="Disable Detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(enabled=False),
        key_response=fk("bin_disable_detections"),
    )

    disable_detections_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Disable Detections Succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("bin_disable_detections"),
            value=False,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    
    rotate_correctly.add_children([
        get_points_1,
        rotate_position,
        get_points_2,
        compare_positions,
        moveto_correct_position,
    ])

    launch_seq.add_children(
        children=[
            goto_centre,
            srv_enable_detections,
            enable_detections_succeeded,
            py_trees.timers.Timer("wait for match", duration=5.0),
            rotate_correctly,
            choose_fish,
            update_selection,
            align_to_target,
            py_trees.timers.Timer("wait to stabilize", duration=5.0),
            set_dropper_actuation,
            fire_dropper_1,
            py_trees.timers.Timer("delay between drops", duration=5.0),
            move_slightly,
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
