import operator

import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from bb_perception_msgs.msg import PointCorrespondencesStamped
from bb_perception_msgs.srv import IMPoseEstimatorToggleTemplate
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
)
from mission_planner_2.trees.auv.bins.bin_selector import create_bin_selector_root
from mission_planner_2.trees.auv.goto import goto

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
TOGGLE_TEMPLATE_TOPIC = "/auv4/bot_cam/image_matching/toggle_template"
TEMPLATE_NAME = "Task03_DropBRUVS.png"
POINT_CORRESPONDENCES_TOPIC = "/auv4/bot_cam/image_matching/point_correspondences"


CAMERA_FRAME = "auv4/bot_cam_optical"
TEMPLATE_FRAME_OPTICAL = "Task03_DropBRUVS_optical"
TEMPLATE_FRAME_OPTICAL_CLUSTERED = "bin/clustered"
TEMPLATE_FRAME_YOLO = "bin/yolo"
TEMPLATE_FRAME_YOLO_CLUSTERED = "bin/yolo/clustered"

ACTUATION_UINT = UInt8(data=6)

CHOICE_KEY = "choice"
POSE_KEY = "pose"

CLUSTERING_DURATION = 20
STABILIZE_DURATION = 5.0
#########################################################################


def create_bin_root():
    """
    Create the root of the bin tree.
    """

    seq_bin_root = py_trees.composites.Sequence(
        name="Bin root",
        memory=True,
    )

    seq_drop_into_bin = py_trees.composites.Sequence(
        name="Drop into bin",
        memory=True,
    )

    # Step 1: Stabilise before starting
    timer_stabilise = py_trees.timers.Timer(
        "Stabilise before task", duration=STABILIZE_DURATION
    )

    # Step 2: Cluster transforms for initial orientation using YOLO
    action_cluster_first = py_trees_ros.action_clients.FromConstant(
        name="Cluster transforms for orientation",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_parent=CAMERA_FRAME,
            in_child=TEMPLATE_FRAME_YOLO,
            out_child=TEMPLATE_FRAME_YOLO_CLUSTERED,
            duration=CLUSTERING_DURATION,
            use_cache=False,
        ),
    )

    # Step 3: Navigate to bin centre
    goto_bin_centre = goto.FromConstant(
        name="Goto bin centre",
        parent_namespace=NAMESPACE,
        pose=create_stamped_pose("bin/centre"),
    )

    # Step 4: Enable image matching detections
    srv_enable_detections = py_trees_ros.service_clients.FromConstant(
        name="Enable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(
            enabled=True,
            camera_frame_id=CAMERA_FRAME,
            template_name=TEMPLATE_NAME,
        ),
        key_response=fk("bin_enable_detections"),
    )

    # Step 5: Verify enable succeeded
    check_enable_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify enable succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("bin_enable_detections"),
            value=True,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    # Step 6: Rotate correctly sequence
    seq_rotate_correctly = py_trees.composites.Sequence(
        name="Rotate correctly",
        memory=True,
    )

    # Step 6a: Get first set of point correspondences
    sub_get_points_first = py_trees_ros.subscribers.ToBlackboard(
        name="Get points first",
        topic_name=POINT_CORRESPONDENCES_TOPIC,
        topic_type=PointCorrespondencesStamped,
        qos_profile=qos_profile_system_default,
        blackboard_variables={fk("points_1"): "object_points"},
    )

    # Step 6b: Rotate to check different orientation
    goto_rotate_position = goto.FromConstant(
        name="Rotate position",
        parent_namespace=NAMESPACE,
        pose=create_stamped_pose("auv4/base_link_ned", yaw=180.0),
    )

    # Step 6c: Wait for stabilisation
    timer_wait_points = py_trees.timers.Timer(
        "Wait for second points",
        duration=STABILIZE_DURATION,
    )

    # Step 6d: Get second set of point correspondences
    sub_get_points_second = py_trees_ros.subscribers.ToBlackboard(
        name="Get points second",
        topic_name=POINT_CORRESPONDENCES_TOPIC,
        topic_type=PointCorrespondencesStamped,
        qos_profile=qos_profile_system_default,
        blackboard_variables={fk("points_2"): "object_points"},
    )

    # Step 6e: Compare orientations and choose best
    set_compare_positions = DynamicSetBlackboard(
        name="Compare positions",
        namespace=NAMESPACE,
        key=["points_1", "points_2"],
        update_key="correct_orientation_pose",
        func=lambda x, y: (
            create_stamped_pose("auv4/base_link_ned", yaw=180.0)
            if len(x.data) > len(y.data)
            else create_stamped_pose("auv4/base_link_ned")
        ),
    )

    # Step 6f: Move to correct orientation
    goto_correct_position = goto.FromBlackboard(
        name="Goto correct position",
        parent_namespace=NAMESPACE,
        pose_key="correct_orientation_pose",
    )

    # Step 7: Get fish/shark choice
    srv_choose_fish = py_trees_ros.service_clients.FromConstant(
        name="Get choice",
        service_name="/auv4/choice/get_is_fish",
        service_type=Trigger,
        service_request=Trigger.Request(),
        key_response=fk(CHOICE_KEY),
    )

    # Step 8: Update pose selection based on choice
    sel_update_selection = create_bin_selector_root(
        choice_key=fk(CHOICE_KEY), pose_key=fk(POSE_KEY)
    )

    # Step 9: Cluster transforms for precise dropping using image matching
    action_cluster_second = py_trees_ros.action_clients.FromConstant(
        name="Cluster transforms for dropping",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_parent=CAMERA_FRAME,
            in_child=TEMPLATE_FRAME_OPTICAL,
            out_child=TEMPLATE_FRAME_OPTICAL_CLUSTERED,
            duration=CLUSTERING_DURATION,
            use_cache=False,
        ),
    )

    # Step 10: Align to precise target
    goto_align_to_target = goto.FromBlackboard(
        name="Align to target",
        parent_namespace=NAMESPACE,
        pose_key=POSE_KEY,
    )

    # Step 11: Set dropper actuation value
    set_dropper_actuation = py_trees.behaviours.SetBlackboardVariable(
        name="Set dropper actuation",
        variable_name=fk("bin_actuation"),
        variable_value=ACTUATION_UINT,
        overwrite=True,
    )

    # Step 12: Fire first dropper
    pub_fire_dropper_first = py_trees_ros.publishers.FromBlackboard(
        name="Fire dropper first",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=fk("bin_actuation"),
    )

    # Step 13: Move slightly for second drop
    goto_move_slightly = goto.FromConstant(
        name="Move slightly",
        parent_namespace=NAMESPACE,
        pose=create_stamped_pose("auv4/base_link_ned", position_x=0.05),
    )

    # Step 14: Fire second dropper
    pub_fire_dropper_second = py_trees_ros.publishers.FromBlackboard(
        name="Fire dropper second",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=fk("bin_actuation"),
    )

    # Step 15: Disable detections
    srv_disable_detections = py_trees_ros.service_clients.FromConstant(
        name="Disable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(enabled=False),
        key_response=fk("bin_disable_detections"),
    )

    # Step 16: Verify disable succeeded
    check_disable_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify disable succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("bin_disable_detections"),
            value=False,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    # Build rotate correctly sequence
    seq_rotate_correctly.add_children(
        [
            sub_get_points_first,
            goto_rotate_position,
            timer_wait_points,
            sub_get_points_second,
            set_compare_positions,
            goto_correct_position,
        ]
    )

    # Build main drop sequence
    seq_drop_into_bin.add_children(
        children=[
            action_cluster_first,
            # goto_bin_centre,
            # srv_enable_detections,
            # check_enable_succeeded,
            # seq_rotate_correctly,
            # srv_choose_fish,
            # sel_update_selection,
            # action_cluster_second,
            # goto_align_to_target,
            # set_dropper_actuation,
            # # pub_fire_dropper_first,
            # # goto_move_slightly,
            # # pub_fire_dropper_second,
            # srv_disable_detections,
            # check_disable_succeeded,
        ],
    )

    # Build root sequence
    seq_bin_root.add_children(
        children=[
            # create_move_to_bin_task_root(),
            timer_stabilise,
            seq_drop_into_bin,
        ]
    )

    return seq_bin_root
