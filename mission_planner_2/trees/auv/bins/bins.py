import operator

import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from bb_perception_msgs.msg import PointCorrespondencesStamped
from bb_perception_msgs.srv import IMPoseEstimatorToggleTemplate
from lifecycle_msgs.srv import ChangeState
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.detection_utils import (
    create_end_vision_req,
    create_start_vision_req,
)
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
VISION_SERVER_TOPIC = "/auv4/bin/manage_nodes"

TOGGLE_TEMPLATE_TOPIC = "/auv4/bin/image_matching/toggle_template"
TEMPLATE_NAME = "Task03_DropBRUVS.png"
ROTATED_TEMPLATE_NAME = "Task03_DropBRUVS_Rotated.png"
POINT_CORRESPONDENCES_TOPIC = "/auv4/bin/image_matching/point_correspondences"

CAMERA_FRAME = "auv4/bot_cam_optical"
TEMPLATE_FRAME_OPTICAL = "Task03_DropBRUVS_optical"
ROTATED_TEMPLATE_FRAME_OPTICAL = "Task03_DropBRUVS_Rotated_optical"
TEMPLATE_FRAME_OPTICAL_CLUSTERED = "bin/clustered"
TEMPLATE_FRAME_YOLO = "bin/yolo"
TEMPLATE_FRAME_YOLO_CLUSTERED = "bin/yolo/clustered"

ACTUATION_UINT = UInt8(data=6)
ACTUATION_INPUT_TOPIC = "/auv4/actuation/input"

CLUSTERING_DURATION = 20
STABILIZE_CONTROLS_DURATION = 10.0

FISH_BIN_FRAME = "bin/fish"
SHARK_BIN_FRAME = "bin/shark"
FISH_BIN_ROTATED_FRAME = "bin/fish/rotated"
SHARK_BIN_ROTATED_FRAME = "bin/shark/rotated"
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_CHOICE_KEY = fk("choice")
_POSE_KEY = fk("pose")
_POINTS_1_KEY = fk("points_1")
_POINTS_2_KEY = fk("points_2")
_START_VISION_KEY = fk("bin_start_vision")
_STOP_VISION_KEY = fk("bin_stop_vision")


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

    # Step 0: Enable vision pipeline
    srv_start_vision = py_trees_ros.service_clients.FromConstant(
        name="Start vision pipeline",
        service_name=VISION_SERVER_TOPIC,
        service_type=ChangeState,
        service_request=create_start_vision_req(),
        key_response=_START_VISION_KEY,
    )
    check_start_vision_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify start vision pipeline succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=_START_VISION_KEY,
            value=True,
            operator=lambda x, y: operator.eq(x.success, y),
        ),
    )
    # Step 1: Stabilise before starting
    timer_stabilise = py_trees.timers.Timer(
        "Stabilise before task", duration=STABILIZE_CONTROLS_DURATION
    )

    # Step 2: Cluster transforms for initial orientation using YOLO
    action_cluster_first = py_trees_ros.action_clients.FromConstant(
        name="Cluster transforms for orientation",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=TEMPLATE_FRAME_YOLO,
            out_children=TEMPLATE_FRAME_YOLO_CLUSTERED,
            duration=CLUSTERING_DURATION,
            use_cache=False,
        ),
    )

    # Step 3: Navigate to bin centre
    goto_bin_centre = goto.FromConstant(
        name="Goto bin centre",
        pose=create_stamped_pose("bin/centre"),
    )

    stabilise_before_matching = py_trees.timers.Timer(
        "Stabilise before matching", duration=STABILIZE_CONTROLS_DURATION
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

    # Step 6a: Get first set of point correspondences
    sub_get_points_first = py_trees_ros.subscribers.ToBlackboard(
        name="Get points first",
        topic_name=POINT_CORRESPONDENCES_TOPIC,
        topic_type=PointCorrespondencesStamped,
        qos_profile=qos_profile_sensor_data,
        blackboard_variables={
            _POINTS_1_KEY: "object_points",
            fk("object_frame_id_1"): "object_frame_id",
        },
    )
    # Check whether the point correspondences are for the newly
    # set template and not from a previous task.
    check_point_correspondences_first = (
        py_trees.behaviours.CheckBlackboardVariableValue(
            name="Check point correspondences first",
            check=py_trees.common.ComparisonExpression(
                variable=fk("object_frame_id_1"),
                value=TEMPLATE_FRAME_OPTICAL,
                operator=operator.eq,
            ),
        )
    )
    sub_get_points_first_sequence = py_trees.composites.Sequence(
        name="Try unrotated template",
        memory=True,
    )
    sub_get_points_first_sequence.add_children(
        children=[
            sub_get_points_first,
            check_point_correspondences_first,
        ],
    )
    sub_get_points_first_sequence_retry = py_trees.decorators.Retry(
        name="Retry get points first",
        child=sub_get_points_first_sequence,
        num_failures=100,
    )

    srv_enable_detections_rotated = py_trees_ros.service_clients.FromConstant(
        name="Enable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(
            enabled=True,
            camera_frame_id=CAMERA_FRAME,
            template_name=ROTATED_TEMPLATE_NAME,
        ),
        key_response=fk("bin_rotated_enable_detections"),
    )

    check_enable_succeeded_rotated = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify enable succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("bin_rotated_enable_detections"),
            value=True,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    # Step 6b: Get second set of point correspondences
    sub_get_points_second = py_trees_ros.subscribers.ToBlackboard(
        name="Get points second",
        topic_name=POINT_CORRESPONDENCES_TOPIC,
        topic_type=PointCorrespondencesStamped,
        qos_profile=qos_profile_sensor_data,
        blackboard_variables={
            _POINTS_2_KEY: "object_points",
            fk("object_frame_id_2"): "object_frame_id",
        },
    )
    # Check whether the point correspondences are for the newly
    # set template and not from a previous task.
    check_point_correspondences_second = (
        py_trees.behaviours.CheckBlackboardVariableValue(
            name="Check point correspondences second",
            check=py_trees.common.ComparisonExpression(
                variable=fk("object_frame_id_2"),
                value=ROTATED_TEMPLATE_FRAME_OPTICAL,
                operator=operator.eq,
            ),
        )
    )
    sub_get_points_second_sequence = py_trees.composites.Sequence(
        name="Get points second sequence",
        memory=True,
    )
    sub_get_points_second_sequence.add_children(
        children=[
            sub_get_points_second,
            check_point_correspondences_second,
        ],
    )
    sub_get_points_second_sequence_retry = py_trees.decorators.Retry(
        name="Try rotated template",
        child=sub_get_points_second_sequence,
        num_failures=100,
    )

    def create_enable_req(points_1, points_2):
        if len(points_1.data) > len(points_2.data):
            template_name = TEMPLATE_NAME
        else:
            template_name = ROTATED_TEMPLATE_NAME

        return IMPoseEstimatorToggleTemplate.Request(
            enabled=True,
            camera_frame_id=CAMERA_FRAME,
            template_name=template_name,
        )

    def create_correct_clustering_goal(points_1, points_2):
        if len(points_1.data) > len(points_2.data):
            template_frame = TEMPLATE_FRAME_OPTICAL
        else:
            template_frame = ROTATED_TEMPLATE_FRAME_OPTICAL

        return create_clustering_goal(
            in_children=template_frame,
            out_children=TEMPLATE_FRAME_OPTICAL_CLUSTERED,
            duration=CLUSTERING_DURATION,
            use_cache=False,
        )

    set_enable_detections_req = DynamicSetBlackboard(
        name="Set enable detections request",
        key=[_POINTS_1_KEY, _POINTS_2_KEY],
        update_key=fk("enable_detections_req"),
        func=create_enable_req,
    )

    set_clustering_goal = DynamicSetBlackboard(
        name="Set clustering goal",
        key=[_POINTS_1_KEY, _POINTS_2_KEY],
        update_key=fk("clustering_goal"),
        func=create_correct_clustering_goal,
    )

    # Step 7: Get fish/shark choice
    srv_choose_fish = py_trees_ros.service_clients.FromConstant(
        name="Get choice",
        service_name="/auv4/choice/get_is_fish",
        service_type=Trigger,
        service_request=Trigger.Request(),
        key_response=_CHOICE_KEY,
    )

    # Step 8: Update pose selection based on choice
    sel_update_selection = create_bin_selector_root(
        choice_key=_CHOICE_KEY,
        pose_key=_POSE_KEY,
        points1_key=_POINTS_1_KEY,
        points2_key=_POINTS_2_KEY,
        fish_bin_frame=FISH_BIN_FRAME,
        shark_bin_frame=SHARK_BIN_FRAME,
        fish_bin_rotated_frame=FISH_BIN_ROTATED_FRAME,
        shark_bin_rotated_frame=SHARK_BIN_ROTATED_FRAME,
    )

    srv_enable_correct_detections = py_trees_ros.service_clients.FromBlackboard(
        "Enable correct detection template",
        service_type=IMPoseEstimatorToggleTemplate,
        service_name=TOGGLE_TEMPLATE_TOPIC,
        key_request=fk("enable_detections_req"),
        key_response=fk("bin_correct_enable_detections"),
    )

    check_enable_succeeded_correct = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify enable succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("bin_correct_enable_detections"),
            value=True,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    action_cluster_second = py_trees_ros.action_clients.FromBlackboard(
        name="Cluster transforms for dropping",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        key=fk("clustering_goal"),
    )

    # Step 10: Align to precise target
    goto_align_to_target = goto.FromBlackboard(
        name="Align to target",
        pose_key=_POSE_KEY,
        anchor_frame_name="auv4/dropper",
    )

    stabilise_before_dropping = py_trees.timers.Timer(
        "Stabilise before dropping", duration=STABILIZE_CONTROLS_DURATION
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
        topic_name=ACTUATION_INPUT_TOPIC,
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=fk("bin_actuation"),
    )

    # Step 14: Fire second dropper
    pub_fire_dropper_second = py_trees_ros.publishers.FromBlackboard(
        name="Fire dropper second",
        topic_name=ACTUATION_INPUT_TOPIC,
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

    # Step 17: End vision pipeline
    srv_end_vision = py_trees_ros.service_clients.FromConstant(
        name="End vision pipeline",
        service_name=VISION_SERVER_TOPIC,
        service_type=ChangeState,
        service_request=create_end_vision_req(),
        key_response=_STOP_VISION_KEY,
    )
    check_end_vision_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify end vision pipeline succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=_STOP_VISION_KEY,
            value=True,
            operator=lambda x, y: operator.eq(x.success, y),
        ),
    )

    # Build main drop sequence
    seq_drop_into_bin.add_children(
        children=[
            srv_start_vision,
            check_start_vision_succeeded,
            action_cluster_first,
            goto_bin_centre,
            stabilise_before_matching,
            srv_enable_detections,
            check_enable_succeeded,
            sub_get_points_first_sequence_retry,
            srv_enable_detections_rotated,
            check_enable_succeeded_rotated,
            sub_get_points_second_sequence_retry,
            set_enable_detections_req,
            set_clustering_goal,
            srv_choose_fish,
            sel_update_selection,
            srv_enable_correct_detections,
            check_enable_succeeded_correct,
            action_cluster_second,
            goto_align_to_target,
            stabilise_before_dropping,
            set_dropper_actuation,
            pub_fire_dropper_first,
            py_trees.timers.Timer(name="Wait between drops", duration=2.0),
            pub_fire_dropper_second,
            srv_disable_detections,
            check_disable_succeeded,
            srv_end_vision,
            check_end_vision_succeeded,
        ],
    )

    # Build root sequence
    seq_bin_root.add_children(
        children=[
            # timer_stabilise,
            seq_drop_into_bin,
        ]
    )

    return seq_bin_root
