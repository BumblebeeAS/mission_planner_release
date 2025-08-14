import operator

import py_trees
from bb_perception_msgs.srv import IMPoseEstimatorToggleTemplate
from lifecycle_msgs.srv import ChangeState

from mission_planner_2.commons import checked_service
from mission_planner_2.commons.blackboard import (
    MultiSetBlackboard,
)
from mission_planner_2.commons.detection_utils import (
    create_end_vision_req,
    create_img_matching_request,
    create_start_vision_req,
)
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.commons.search import create_search_front_root
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.torpedo.move_and_shoot_seq import (
    create_move_and_shoot_generator,
)
from mission_planner_2.trees.auv.torpedo.point_correspondences_check import (
    create_point_correspondences_check_root,
)

# Generate namespace automatically from file path DONT set manually
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
SELECTED_TEMPLATE = 2  # MUST be 1 or 2

VISION_SERVER_TOPIC = "/auv4/torpedo/manage_nodes"
TOGGLE_TEMPLATE_TOPIC = "/auv4/torpedo/image_matching/toggle_template"
POINT_CORRESPONDENCES_TOPIC = "/auv4/torpedo/image_matching/point_correspondences"

CAMERA_FRAME = "auv4/front_cam_optical"
TORPEDO_SHOOTER_LEFT_FRAME = "auv4/torpedo_shooter_left"
TORPEDO_SHOOTER_RIGHT_FRAME = "auv4/torpedo_shooter_right"
# TORPEDO_SHOOTER_LEFT_FRAME = "auv4/front_cam_optical"
# TORPEDO_SHOOTER_RIGHT_FRAME = "auv4/front_cam_optical"
TEMPLATE_FRAME_YOLO = "torpedo/yolo"
TEMPLATE_FRAME_YOLO_CLUSTERED = "torpedo/yolo/clustered"

CENTRE_VIEW_FRAME = "torpedo/centre/view"

ACTUATION_TOPIC_LEFT = "/auv4/actuation/torpedo/left"
ACTUATION_TOPIC_RIGHT = "/auv4/actuation/torpedo/right"

DISTANCE_THRESHOLD = 0.025
YAW_THRESHOLD = 1.0

CLUSTER_DURATION = 4
REALIGN_CLUSTER_DURATION = 4
STABILIZE_DURATION = 3
WAIT_AFTER_FIRE_DURATION = 3.0
NUM_RETRIES = 3
SEARCH_DEPTH = 1.2

TORPEDO_TEMPLATE_1 = "Task04_Tagging_01.png"
TORPEDO_TEMPLATE_2 = "Task04_Tagging_02.png"

TEMPLATE_FRAME_OPTICAL_1 = "Task04_Tagging_01_optical"
TEMPLATE_FRAME_OPTICAL_2 = "Task04_Tagging_02_optical"

TEMPLATE_FRAME_CLUSTERED_1 = "torpedo_1"
TEMPLATE_FRAME_CLUSTERED_2 = "torpedo_2"

FISH_SHOOT_FRAME_1 = "torpedo_1/fish/view"
FISH_SHOOT_FRAME_2 = "torpedo_2/fish/view"

SHARK_SHOOT_FRAME_1 = "torpedo_1/shark/view"
SHARK_SHOOT_FRAME_2 = "torpedo_2/shark/view"

POINT_CORRESPONDENCES_TOPIC = "/auv4/torpedo/image_matching/point_correspondences"
SHOOT_REPEATS = 2
MAX_ALIGN_FAILURE = 5
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_POSE_KEY = fk("pose")
_POSE_FRAME_KEY = fk("pose_frame")
_ANCHOR_FRAME_KEY = fk("anchor_frame")
_CORRECT_TEMPLATE_KEY = fk("correct_template")
_CORRECT_REQ_KEY = fk("correct_req")
_CORRECT_FISH_KEY = fk("correct_fish")
_CORRECT_SHARK_KEY = fk("correct_shark")
_CORRECT_CLUSTERED_KEY = fk("correct_clustered")


def create_torpedo_root():
    """
    Create the root of the torpedo tree.

    1 - move to task - in progress
    2 - make choice
    3 - enable detections
    4 - save tf + align to target
    5 - launch torpedo1
    6 - go to old tf saved in 4 to reset position
    7 - align to target 2
    8 - launch torpedo2
    9 - disable detections

    For manual testing with dummy tfs (if you are too lazy to keep running image matching).
    ros2 run tf2_ros static_transform_publisher -3.3 0 -0.9 0 0 1.57 world fake_det # usually the pose the detection gives
    ros2 run tf2_ros static_transform_publisher 0.3 0 0.6 0 1.57 1.57 fake_det hole
    """
    # if SELECTED_TEMPLATE == 1:
    #     template_name = "Task04_Tagging_01.png"
    #     template_frame_optical = "Task04_Tagging_01_optical"
    #     template_frame_optical_clustered = "torpedo_1"
    #     fish_shoot_frame = "torpedo_1/fish/view"
    #     shark_shoot_frame = "torpedo_1/shark/view"
    # elif SELECTED_TEMPLATE == 2:
    #     template_name = "Task04_Tagging_02.png"
    #     template_frame_optical = "Task04_Tagging_02_optical"
    #     template_frame_optical_clustered = "torpedo_2"
    #     fish_shoot_frame = "torpedo_2/fish/view"
    #     shark_shoot_frame = "torpedo_2/shark/view"
    # else:
    #     raise ValueError("Invalid template selected, must be 1 or 2")

    move_and_shoot_gen = create_move_and_shoot_generator(
        anchor_frame_key=_ANCHOR_FRAME_KEY,
        torpedo_shooter_left_frame=TORPEDO_SHOOTER_LEFT_FRAME,
        torpedo_shooter_right_frame=TORPEDO_SHOOTER_RIGHT_FRAME,
        choice_key="/global/choice_is_fish",
        pose_key=_POSE_KEY,
        pose_frame_key=_POSE_FRAME_KEY,
        fish_shoot_frame_key=_CORRECT_FISH_KEY,
        shark_shoot_frame_key=_CORRECT_SHARK_KEY,
        template_frame_optical_key=_CORRECT_TEMPLATE_KEY,
        template_frame_optical_clustered_key=_CORRECT_CLUSTERED_KEY,
        cluster_duration=CLUSTER_DURATION,
        realign_cluster_duration=REALIGN_CLUSTER_DURATION,
        actuation_topic_left=ACTUATION_TOPIC_LEFT,
        actuation_topic_right=ACTUATION_TOPIC_RIGHT,
        distance_threshold=DISTANCE_THRESHOLD,
        yaw_threshold=YAW_THRESHOLD,
        retries=MAX_ALIGN_FAILURE,
        stabilization_duration=2.5,
        num_retries_clustering=NUM_RETRIES,
        wait_after_fire_duration=WAIT_AFTER_FIRE_DURATION,
        shoot_repeats=SHOOT_REPEATS,
    )

    seq_launch_torpedo = py_trees.composites.Sequence(
        name="Launch torpedo",
        memory=True,
    )

    srv_start_vision = checked_service.FromConstant(
        name="Start vision",
        service_type=ChangeState,
        service_name=VISION_SERVER_TOPIC,
        service_request=create_start_vision_req(),
        check_func=lambda x: x.success,
    )

    retry_start_vision = py_trees.decorators.Retry(
        name="Retry start vision",
        child=srv_start_vision,
        num_failures=3,
    )

    seq_search = create_search_front_root(
        object_frame=TEMPLATE_FRAME_YOLO,
        object_frame_clustered=TEMPLATE_FRAME_YOLO_CLUSTERED,
        wait_between_moves=2.0,
        search_depth=SEARCH_DEPTH,
    )

    # use this if not seq_search
    # cluster_board_centre = shared_action_client.FromConstant(
    #     name="Cluster centre (debug)",
    #     shared_action=SharedAction.CLUSTER,
    #     action_goal=create_clustering_goal(
    #         in_children=TEMPLATE_FRAME_YOLO,
    #         out_children=TEMPLATE_FRAME_YOLO_CLUSTERED,
    #         duration=CLUSTER_DURATION,
    #     ),
    # )

    goto_torp_centre = goto.FromConstant(
        name="Goto torp centre",
        pose=create_stamped_pose(CENTRE_VIEW_FRAME),
    )

    stabilise_before_matching = py_trees.timers.Timer(
        name="Stabilise before match",
        duration=STABILIZE_DURATION,
    )

    seq_check_point_correspondences = create_point_correspondences_check_root(
        toggle_template_topic=TOGGLE_TEMPLATE_TOPIC,
        camera_frame=CAMERA_FRAME,
        template_frame_1=TORPEDO_TEMPLATE_1,
        template_frame_2=TORPEDO_TEMPLATE_2,
        template_frame_optical_1=TEMPLATE_FRAME_OPTICAL_1,
        template_frame_optical_2=TEMPLATE_FRAME_OPTICAL_2,
        num_retries=NUM_RETRIES,
        points_correspondences_topic=POINT_CORRESPONDENCES_TOPIC,
        correct_template_key=_CORRECT_TEMPLATE_KEY,
    )

    sel_correct_stuff = py_trees.composites.Selector(
        name="Select correct frames to set",
        memory=True,
    )

    seq_set_template_1 = py_trees.composites.Sequence(
        name="Set for template 1",
        memory=True,
    )

    check_template = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check template",
        check=py_trees.common.ComparisonExpression(
            variable=_CORRECT_TEMPLATE_KEY,
            value=TEMPLATE_FRAME_OPTICAL_1,
            operator=operator.eq,
        ),
    )

    set_multi_template_1 = MultiSetBlackboard(
        name="Set template 1 stuff",
        keys=[
            _CORRECT_REQ_KEY,
            _CORRECT_FISH_KEY,
            _CORRECT_SHARK_KEY,
            _CORRECT_CLUSTERED_KEY,
        ],
        values=[
            create_img_matching_request(
                enable=True,
                camera_frame_id=CAMERA_FRAME,
                template_name=TORPEDO_TEMPLATE_1,
            ),
            FISH_SHOOT_FRAME_1,
            SHARK_SHOOT_FRAME_1,
            TEMPLATE_FRAME_CLUSTERED_1,
        ],
        overwrite=True,
    )

    seq_set_template_1.add_children(
        [
            check_template,
            set_multi_template_1,
        ]
    )

    set_multi_template_2 = MultiSetBlackboard(
        name="Set template 2 stuff",
        keys=[
            _CORRECT_REQ_KEY,
            _CORRECT_FISH_KEY,
            _CORRECT_SHARK_KEY,
            _CORRECT_CLUSTERED_KEY,
        ],
        values=[
            create_img_matching_request(
                enable=True,
                camera_frame_id=CAMERA_FRAME,
                template_name=TORPEDO_TEMPLATE_2,
            ),
            FISH_SHOOT_FRAME_2,
            SHARK_SHOOT_FRAME_2,
            TEMPLATE_FRAME_CLUSTERED_2,
        ],
        overwrite=True,
    )

    sel_correct_stuff.add_children(
        [
            seq_set_template_1,
            set_multi_template_2,
        ]
    )

    srv_enable_correct_detections_frame = checked_service.FromBlackboard(
        name="Enable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        key_request=_CORRECT_REQ_KEY,
        check_func=lambda x: x.new_state,  # check if the service call was successful
    )

    retry_enable_detections = py_trees.decorators.Retry(
        name="retry enable correct srv",
        child=srv_enable_correct_detections_frame,
        num_failures=NUM_RETRIES,
    )

    move_and_shoot_first = move_and_shoot_gen(first=True)

    goto_back_centre = goto.FromConstant(
        name="Go back to centre",
        pose=create_stamped_pose(CENTRE_VIEW_FRAME),
        anchor_frame_name=CAMERA_FRAME,
    )

    move_and_shoot_second = move_and_shoot_gen(first=False)

    seq_stop_vision = py_trees.composites.Sequence(
        name="Stop vision",
        memory=True,
    )

    srv_disable_detections = checked_service.FromConstant(
        name="Disable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=create_img_matching_request(
            enable=False,
            camera_frame_id=CAMERA_FRAME,
            template_name="",
        ),
        check_func=lambda x: not x is not None and x.new_state == False,
    )

    retry_disable_detections = py_trees.decorators.Retry(
        name="Retry Disable Detections",
        child=srv_disable_detections,
        num_failures=NUM_RETRIES,
    )

    force_success_disable_detections = py_trees.decorators.FailureIsSuccess(
        name="Force success disable detections",
        child=retry_disable_detections,
    )

    srv_end_vision = checked_service.FromConstant(
        name="End vision",
        service_type=ChangeState,
        service_name=VISION_SERVER_TOPIC,
        service_request=create_end_vision_req(),
        check_func=lambda x: x.success,
    )

    retry_end_vision = py_trees.decorators.Retry(
        name="Retry End Vision",
        child=srv_end_vision,
        num_failures=NUM_RETRIES,
    )

    force_success_end_vision = py_trees.decorators.FailureIsSuccess(
        name="Force success end vision",
        child=retry_end_vision,
    )

    seq_stop_vision.add_children(
        children=[
            force_success_disable_detections,
            force_success_end_vision,
        ],
    )

    seq_launch_torpedo.add_children(
        children=[
            retry_start_vision,
            seq_search,
            # cluster_board_centre,
            goto_torp_centre,
            # stabilise_before_matching,
            seq_check_point_correspondences,
            sel_correct_stuff,
            retry_enable_detections,
            move_and_shoot_first,
            goto_back_centre,
            move_and_shoot_second,
            seq_stop_vision,
        ],
    )

    return seq_launch_torpedo
