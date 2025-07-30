import py_trees
import py_trees_ros
from bb_perception_msgs.srv import IMPoseEstimatorToggleTemplate
from lifecycle_msgs.srv import ChangeState
from std_srvs.srv import Trigger

from mission_planner_2.commons import checked_service
from mission_planner_2.commons.detection_utils import (
    create_end_vision_req,
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

# Generate namespace automatically from file path DONT set manually
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
SELECTED_TEMPLATE = 1  # MUST be 1 or 2

VISION_SERVER_TOPIC = "/auv4/torpedo/manage_nodes"
TOGGLE_TEMPLATE_TOPIC = "/auv4/torpedo/image_matching/toggle_template"
CAMERA_FRAME = "auv4/front_cam_optical"
TORPEDO_SHOOTER_LEFT_FRAME = "auv4/torpedo_shooter_left"
TORPEDO_SHOOTER_RIGHT_FRAME = "auv4/torpedo_shooter_right"
TEMPLATE_FRAME_YOLO = "torpedo/yolo"
TEMPLATE_FRAME_YOLO_CLUSTERED = "torpedo/yolo/clustered"

CENTRE_VIEW_FRAME = "torpedo/centre/view"

ACTUATION_TOPIC_LEFT = "/auv4/actuation/torpedo/left"
ACTUATION_TOPIC_RIGHT = "/auv4/actuation/torpedo/right"

CLUSTER_DURATION = 4
REALIGN_CLUSTER_DURATION = 2
STABILIZE_DURATION = 3
NUM_RETRIES = 3
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_CHOICE_KEY = fk("choice")
_POSE_KEY = fk("pose")
_POSE_FRAME_KEY = fk("pose_frame")
_ANCHOR_FRAME_KEY = fk("anchor_frame")


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
    if SELECTED_TEMPLATE == 1:
        template_name = "Task04_Tagging_01.png"
        template_frame_optical = "Task04_Tagging_01_optical"
        template_frame_optical_clustered = "torpedo_1"
        fish_shoot_frame = "torpedo_1/fish/view"
        shark_shoot_frame = "torpedo_1/shark/view"
    elif SELECTED_TEMPLATE == 2:
        template_name = "Task04_Tagging_02.png"
        template_frame_optical = "Task04_Tagging_02_optical"
        template_frame_optical_clustered = "torpedo_2"
        fish_shoot_frame = "torpedo_2/fish/view"
        shark_shoot_frame = "torpedo_2/shark/view"
    else:
        raise ValueError("Invalid template selected, must be 1 or 2")

    move_and_shoot_gen = create_move_and_shoot_generator(
        anchor_frame_key=_ANCHOR_FRAME_KEY,
        torpedo_shooter_left_frame=TORPEDO_SHOOTER_LEFT_FRAME,
        torpedo_shooter_right_frame=TORPEDO_SHOOTER_RIGHT_FRAME,
        choice_key=_CHOICE_KEY,
        pose_key=_POSE_KEY,
        pose_frame_key=_POSE_FRAME_KEY,
        fish_shoot_frame=fish_shoot_frame,
        shark_shoot_frame=shark_shoot_frame,
        template_frame_optical=template_frame_optical,
        template_frame_optical_clustered=template_frame_optical_clustered,
        cluster_duration=CLUSTER_DURATION,
        realign_cluster_duration=REALIGN_CLUSTER_DURATION,
        actuation_topic_left=ACTUATION_TOPIC_LEFT,
        actuation_topic_right=ACTUATION_TOPIC_RIGHT,
        distance_threshold=0.025,
        yaw_threshold=1.0,
        retries=8,
        stabilization_duration=2.5,
        num_retries_clustering=NUM_RETRIES,
    )

    seq_torpedo_root = py_trees.composites.Sequence(
        name="Torpedo root",
        memory=True,
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
        wait_between_moves=7.0,
    )

    # use this if not seq_search
    # cluster_board_centre = py_trees_ros.action_clients.FromConstant(
    #     name="Cluster centre",
    #     action_type=ClusterTf,
    #     action_name="/auv4/cluster_tf",
    #     action_goal=create_clustering_goal(
    #         in_children=TEMPLATE_FRAME_YOLO,
    #         out_children=TEMPLATE_FRAME_YOLO_CLUSTERED,
    #         duration=CLUSTER_DURATION,
    #         use_cache=False,
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

    srv_get_choice = py_trees_ros.service_clients.FromConstant(
        name="Get choice",
        service_name="/auv4/choice/get_is_fish",
        service_type=Trigger,
        service_request=Trigger.Request(),
        key_response=_CHOICE_KEY,
    )

    srv_enable_detections = checked_service.FromConstant(
        name="Enable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(
            enabled=True, template_name=template_name
        ),
        key_response=fk("torpedo_enable_detections"),
        check_func=lambda x: x.new_state,  # check if the service call was successful
    )

    retry_enable_detections = py_trees.decorators.Retry(
        name="Retry enable detections",
        child=srv_enable_detections,
        num_failures=NUM_RETRIES,
    )
    move_and_shoot_first = move_and_shoot_gen(first=True)

    goto_back_centre = goto.FromConstant(
        name="Go back to centre",
        pose=create_stamped_pose(CENTRE_VIEW_FRAME),
        anchor_frame_name=CAMERA_FRAME,
    )

    move_and_shoot_second = move_and_shoot_gen(first=False)

    srv_disable_detections = checked_service.FromConstant(
        name="Disable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(enabled=False),
        check_func=lambda x: not x is not None and x.new_state == False,
    )

    retry_disable_detections = py_trees.decorators.FailureIsSuccess(
        name="Force success after retry disable detections",
        child=py_trees.decorators.Retry(
            name="Retry Disable Detections",
            child=srv_disable_detections,
            num_failures=NUM_RETRIES,
        ),
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

    seq_launch_torpedo.add_children(
        children=[
            srv_get_choice,
            retry_start_vision,
            seq_search,
            # cluster_board_centre,
            goto_torp_centre,
            # stabilise_before_matching,
            retry_enable_detections,
            move_and_shoot_first,
            goto_back_centre,
            move_and_shoot_second,
            retry_disable_detections,
            retry_end_vision,
        ],
    )

    seq_torpedo_root.add_children(
        children=[
            py_trees.timers.Timer("Stabilise before task", duration=STABILIZE_DURATION),
            seq_launch_torpedo,
        ]
    )

    return seq_torpedo_root
