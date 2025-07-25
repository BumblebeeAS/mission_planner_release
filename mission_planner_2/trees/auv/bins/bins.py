import operator

import py_trees
import py_trees_ros
from bb_perception_msgs.msg import PointCorrespondencesStamped
from bb_perception_msgs.srv import IMPoseEstimatorToggleTemplate
from lifecycle_msgs.srv import ChangeState
from mission_planner_2.commons import cache_tf, shared_action_client
from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.cluster_goto import create_goto_cluster_from_bb_root
from mission_planner_2.commons.detection_utils import (
    create_end_vision_req,
    create_start_vision_req,
)
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.node_registry import SharedAction
from mission_planner_2.commons.pose_utils import within_threshold_xyz
from mission_planner_2.commons.search import create_search_bot_layered_square_root
from mission_planner_2.trees.auv.bins.helpers import find_acute_angle
from mission_planner_2.trees.auv.bins.template_selector import (
    create_template_selector_root,
)
from mission_planner_2.trees.auv.goto import goto
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
VISION_SERVER_TOPIC = "/auv4/bin/manage_nodes"

TOGGLE_TEMPLATE_TOPIC = "/auv4/bin/image_matching/toggle_template"
TEMPLATE_NAME = "Task03_DropBRUVS.png"
ROTATED_TEMPLATE_NAME = "Task03_DropBRUVS_Rotated.png"
POINT_CORRESPONDENCES_TOPIC = "/auv4/bin/image_matching/point_correspondences"

BASE_LINK_FRAME = "auv4/base_link_ned"
CAMERA_FRAME = "auv4/bot_cam_optical"
TEMPLATE_FRAME_OPTICAL = "Task03_DropBRUVS_optical"
ROTATED_TEMPLATE_FRAME_OPTICAL = "Task03_DropBRUVS_Rotated_optical"
TEMPLATE_FRAME_OPTICAL_CLUSTERED = "bin/clustered"
TEMPLATE_FRAME_YOLO = "bin/yolo"
TEMPLATE_FRAME_YOLO_CLUSTERED = "bin/yolo/clustered"

ACTUATION_TOPIC = "/auv4/actuation/dropper"
ACTUATION_UINT = UInt8(data=6)

NUM_SQUARES = 1
OFFSET_COEFF = 1.0
CLUSTERING_DURATION = 4
REALIGN_CLUSTER_DURATION = 2
STABILIZE_CONTROLS_DURATION = 5.0
RETRIES = 5

FISH_BIN_FRAME = "bin/fish"
SHARK_BIN_FRAME = "bin/shark"
ROTATED_FISH_BIN_FRAME = "bin/fish/rotated"
ROTATED_SHARK_BIN_FRAME = "bin/shark/rotated"

FISH_BIN_VIEW_FRAME = "bin/fish/view"
SHARK_BIN_VIEW_FRAME = "bin/shark/view"
FISH_BIN_VIEW_ROTATED_FRAME = "bin/fish/rotated/view"
SHARK_BIN_VIEW_ROTATED_FRAME = "bin/shark/rotated/view"

SEARCH_PATTERN = [
    {"x": 0.0, "y": -0.25, "z": 0.0},
    {"x": 0.5, "y": 0.0, "z": 0.0},
    {"x": 0.0, "y": 0.5, "z": 0.0},
    {"x": -0.5, "y": 0.0, "z": 0.0},
]
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_CHOICE_KEY = fk("choice")
_POSE_KEY = fk("pose")
_GOTO_FRAME_KEY = fk("goto_frame")
_ANCHOR_FRAME_KEY = fk("anchor_frame")
_POINTS_1_KEY = fk("points_1")
_POINTS_2_KEY = fk("points_2")
_START_VISION_KEY = fk("bin_start_vision")
_STOP_VISION_KEY = fk("bin_stop_vision")
_IS_ROTATED_KEY = fk("is_rotated")
_CLUSTERING_GOAL_KEY = fk("clustering_goal")
_BIN_ENABLE_DETECTIONS_KEY = fk("bin_enable_detections")
_BIN_ROTATED_ENABLE_DETECTIONS_KEY = fk("bin_rotated_enable_detections")
_BIN_CORRECT_DETECTIONS_REQ_KEY = fk("enable_correct_detections_req")
_BIN_CORRECT_ENABLE_DETECTIONS_KEY = fk("bin_correct_enable_detections")
_BIN_DISABLE_DETECTIONS_KEY = fk("bin_disable_detections")
_BIN_CENTRE_TF_KEY = fk("bin_centre_tf")
_BIN_CENTRE_ACUTE_POSE_KEY = fk("bin_centre_acute_pose")


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

    # Step -1: Get fish choice
    srv_get_fish_choice = py_trees_ros.service_clients.FromConstant(
        name="Get fish choice",
        service_type=Trigger,
        service_name="/auv4/choice/get_is_fish",
        service_request=Trigger.Request(),
        key_response=_CHOICE_KEY,
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

    seq_search = create_search_bot_layered_square_root(
        fwd=1.0,
        back=0.3,
        left=1.0,
        right=1.0,
        num_squares=NUM_SQUARES,
        object_frame=TEMPLATE_FRAME_YOLO,
        object_frame_clustered=TEMPLATE_FRAME_YOLO_CLUSTERED,
        offset_coeff=OFFSET_COEFF,
        wait_between_moves=3.0,
    )

    extract_tf = cache_tf.ToBlackboard(
        name="Extract movement to bin centre",
        variable_name=_BIN_CENTRE_TF_KEY,
        start="auv4/base_link_ned",
        end="bin/centre/view",
    )

    calculate_acute_pose = DynamicSetBlackboard(
        name="Calculate acute pose to bin centre",
        key=_BIN_CENTRE_TF_KEY,
        update_key=_BIN_CENTRE_ACUTE_POSE_KEY,
        func=find_acute_angle,
    )

    # Step 2: Navigate to bin centre
    goto_bin_centre = goto.FromBlackboard(
        name="Goto bin centre",
        pose_key=_BIN_CENTRE_ACUTE_POSE_KEY,
    )

    stabilise = py_trees.timers.Timer("Stabilise", duration=STABILIZE_CONTROLS_DURATION)

    # Step 3: Enable image matching detections
    srv_enable_detections = py_trees_ros.service_clients.FromConstant(
        name="Enable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(
            enabled=True,
            camera_frame_id=CAMERA_FRAME,
            template_name=TEMPLATE_NAME,
        ),
        key_response=_BIN_ENABLE_DETECTIONS_KEY,
    )

    # Step 4: Verify enable succeeded
    check_enable_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify enable succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=_BIN_ENABLE_DETECTIONS_KEY,
            value=True,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    # Step 5a: Get first set of point correspondences
    sub_get_points_first = py_trees_ros.subscribers.ToBlackboard(
        name="Get points",
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
            name="Check point correspondences",
            check=py_trees.common.ComparisonExpression(
                variable=fk("object_frame_id_1"),
                value=TEMPLATE_FRAME_OPTICAL,
                operator=operator.eq,
            ),
        )
    )

    seq_get_points_first = py_trees.composites.Sequence(
        name="Seq unrotated template",
        memory=True,
    )

    seq_get_points_first.add_children(
        children=[
            sub_get_points_first,
            check_point_correspondences_first,
        ],
    )

    sub_get_points_first_sequence_retry = py_trees.decorators.Retry(
        name="Try unrotated template",
        child=seq_get_points_first,
        num_failures=100,
    )

    srv_enable_detections_rotated = py_trees_ros.service_clients.FromConstant(
        name="Enable detections (rotated)",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(
            enabled=True,
            camera_frame_id=CAMERA_FRAME,
            template_name=ROTATED_TEMPLATE_NAME,
        ),
        key_response=_BIN_ROTATED_ENABLE_DETECTIONS_KEY,
    )

    check_enable_succeeded_rotated = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify enable succeeded (rotated)",
        check=py_trees.common.ComparisonExpression(
            variable=_BIN_ROTATED_ENABLE_DETECTIONS_KEY,
            value=True,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    # Step 5b: Get second set of point correspondences
    sub_get_points_second = py_trees_ros.subscribers.ToBlackboard(
        name="Get points (rotated)",
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
            name="Check point correspondences (rotated)",
            check=py_trees.common.ComparisonExpression(
                variable=fk("object_frame_id_2"),
                value=ROTATED_TEMPLATE_FRAME_OPTICAL,
                operator=operator.eq,
            ),
        )
    )

    seq_get_points_rotated = py_trees.composites.Sequence(
        name="Seq rotated template",
        memory=True,
    )

    seq_get_points_rotated.add_children(
        children=[
            sub_get_points_second,
            check_point_correspondences_second,
        ],
    )

    sub_get_points_second_sequence_retry = py_trees.decorators.Retry(
        name="Try rotated template",
        child=seq_get_points_rotated,
        num_failures=100,
    )

    sel_update_template = create_template_selector_root(
        points_1_key=_POINTS_1_KEY,
        points_2_key=_POINTS_2_KEY,
        is_rotated_key=_IS_ROTATED_KEY,
        bin_correct_detections_req_key=_BIN_CORRECT_DETECTIONS_REQ_KEY,
        camera_frame=CAMERA_FRAME,
        template_name=TEMPLATE_NAME,
        rotated_template_name=ROTATED_TEMPLATE_NAME,
        clustering_goal_key=_CLUSTERING_GOAL_KEY,
        template_frame_optical=TEMPLATE_FRAME_OPTICAL,
        rotated_template_frame_optical=ROTATED_TEMPLATE_FRAME_OPTICAL,
        template_frame_optical_clustered=TEMPLATE_FRAME_OPTICAL_CLUSTERED,
        clustering_duration=CLUSTERING_DURATION,
        choice_key=_CHOICE_KEY,
        pose_key=_POSE_KEY,
        goto_frame_key=_GOTO_FRAME_KEY,
        fish_bin_view_frame=FISH_BIN_VIEW_FRAME,
        fish_bin_view_rotated_frame=FISH_BIN_VIEW_ROTATED_FRAME,
        shark_bin_view_frame=SHARK_BIN_VIEW_FRAME,
        shark_bin_view_rotated_frame=SHARK_BIN_VIEW_ROTATED_FRAME,
    )

    srv_enable_correct_detections = py_trees_ros.service_clients.FromBlackboard(
        "Enable correct detection template",
        service_type=IMPoseEstimatorToggleTemplate,
        service_name=TOGGLE_TEMPLATE_TOPIC,
        key_request=_BIN_CORRECT_DETECTIONS_REQ_KEY,
        key_response=_BIN_CORRECT_ENABLE_DETECTIONS_KEY,
    )

    check_enable_succeeded_correct = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify enable correct succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=_BIN_CORRECT_ENABLE_DETECTIONS_KEY,
            value=True,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    action_cluster_for_goto = shared_action_client.FromBlackboard(
        name="Cluster transforms for dropping",
        shared_action=SharedAction.CLUSTER,
        key=_CLUSTERING_GOAL_KEY,
    )

    action_cluster_for_goto_check = shared_action_client.FromBlackboard(
        name="Cluster transforms for dropping",
        shared_action=SharedAction.CLUSTER,
        key=_CLUSTERING_GOAL_KEY,
    )

    # Step 10: Align to precise target
    goto_align_to_target = goto.FromBlackboard(
        name="Align to target",
        pose_key=_POSE_KEY,
        anchor_frame_name="auv4/dropper",
    )

    set_anchor_frame = py_trees.behaviours.SetBlackboardVariable(
        name="Set anchor frame",
        variable_name=_ANCHOR_FRAME_KEY,
        variable_value="auv4/dropper",
        overwrite=True,
    )

    # distance_threshold=0.05
    seq_goto_cluster = py_trees.decorators.FailureIsSuccess(
        name="Goto cluster",
        child=create_goto_cluster_from_bb_root(
            cluster_node=action_cluster_for_goto,
            cluster_node_check=action_cluster_for_goto_check,
            goto_node=goto_align_to_target,
            retries=RETRIES,
            start_frame_keys=[_ANCHOR_FRAME_KEY],
            goto_pose_frame_key=_GOTO_FRAME_KEY,
            within_threshold_list=[
                within_threshold_xyz(0.05),
            ],
        ),
    )

    # Uncomment the following lines if you want to use the TF-based goto cluster
    # seq_goto_cluster = create_goto_cluster_from_bb_tf_tf_root(
    #     cluster_node=action_cluster_for_goto,
    #     cluster_node_check=action_cluster_for_goto_check,
    #     goto_node=goto_align_to_target,
    #     distance_threshold=0.05,
    #     retries=3,
    #     tf_frame_key=_GOTO_FRAME_KEY,
    #     within_threshold=within_threshold_dist,
    # )

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
    pub_fire_dropper_first = py_trees_ros.service_clients.FromConstant(
        name="Fire dropper first",
        service_name=ACTUATION_TOPIC,
        service_type=Trigger,
        service_request=Trigger.Request(),
    )

    # Step 13: Fire second dropper
    pub_fire_dropper_second = py_trees_ros.service_clients.FromConstant(
        name="Fire dropper second",
        service_name=ACTUATION_TOPIC,
        service_type=Trigger,
        service_request=Trigger.Request(),
    )

    # Step 14: Disable detections
    srv_disable_detections = py_trees_ros.service_clients.FromConstant(
        name="Disable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(enabled=False),
        key_response=_BIN_DISABLE_DETECTIONS_KEY,
    )

    # Step 15: Verify disable succeeded
    check_disable_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify disable succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=_BIN_DISABLE_DETECTIONS_KEY,
            value=False,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    # Step 16: End vision pipeline
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
            srv_get_fish_choice,
            srv_start_vision,
            check_start_vision_succeeded,
            seq_search,
            extract_tf,
            calculate_acute_pose,
            goto_bin_centre,
            stabilise,
            srv_enable_detections,
            check_enable_succeeded,
            sub_get_points_first_sequence_retry,
            srv_enable_detections_rotated,
            check_enable_succeeded_rotated,
            sub_get_points_second_sequence_retry,
            sel_update_template,
            srv_enable_correct_detections,
            check_enable_succeeded_correct,
            set_anchor_frame,
            seq_goto_cluster,
            set_dropper_actuation,
            pub_fire_dropper_first,
            py_trees.timers.Timer(name="Wait between drops", duration=3.5),
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
            # create_move_to_bin_task_root(),
            seq_drop_into_bin,
        ]
    )

    return seq_bin_root
