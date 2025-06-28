import operator

import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from bb_perception_msgs.srv import IMPoseEstimatorToggleTemplate
from lifecycle_msgs.srv import ChangeState
from std_srvs.srv import Trigger

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
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.torpedo.tf_selector import create_tf_selector_root

# Generate namespace automatically from file path DONT set manually
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
SELECTED_TEMPLATE = 1  # MUST be 1 or 2

VISION_SERVER_TOPIC = "/auv4/torpedo/manage_nodes"
TOGGLE_TEMPLATE_TOPIC = "/auv4/torpedo/image_matching/toggle_template"
CAMERA_FRAME = "auv4/front_cam_optical"
TORPEDO_SHOOTER_TOP_FRAME = "auv4/torpedo_shooter_top"
TORPEDO_SHOOTER_BOT_FRAME = "auv4/torpedo_shooter_bot"
TEMPLATE_FRAME_YOLO = "torpedo/yolo"
TEMPLATE_FRAME_YOLO_CLUSTERED = "torpedo/yolo/clustered"

ACTUATION_TOPIC_TOP = "/auv4/actuation/torpedo/top"
ACTUATION_TOPIC_BOT = "/auv4/actuation/torpedo/bot"

CLUSTER_DURATION = 10
STABILIZE_DURATION = 10
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_CHOICE_KEY = fk("choice")
_POSE_KEY = fk("pose")
_GO_BACK_POSE_KEY = fk("go_back_pose")
_START_VISION_KEY = fk("torp_start_vision")
_STOP_VISION_KEY = fk("torp_stop_vision")


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

    seq_torpedo_root = py_trees.composites.Sequence(
        name="Torpedo root",
        memory=True,
    )

    seq_launch_torpedo = py_trees.composites.Sequence(
        name="Launch torpedo",
        memory=True,
    )

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
            operator=lambda x, y: x.success == y,
        ),
    )

    cluster_board_centre = py_trees_ros.action_clients.FromConstant(
        name="Cluster centre",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=TEMPLATE_FRAME_YOLO,
            out_children=TEMPLATE_FRAME_YOLO_CLUSTERED,
            duration=CLUSTER_DURATION,
            use_cache=False,
        ),
    )

    goto_torp_centre = goto.FromConstant(
        name="Goto torp centre",
        pose=create_stamped_pose("torpedo/centre/view"),
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

    srv_enable_detections = py_trees_ros.service_clients.FromConstant(
        name="Enable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(
            enabled=True, template_name=template_name
        ),
        key_response=fk("torpedo_enable_detections"),
    )

    # check srv call succeeded from the BB
    check_enable_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check enable succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("torpedo_enable_detections"),
            value=True,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    cluster_first = py_trees_ros.action_clients.FromConstant(
        name="Cluster the transforms before first shot",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=template_frame_optical,
            out_children=template_frame_optical_clustered,
            duration=CLUSTER_DURATION,
            use_cache=False,
        ),
    )

    # we call fk(<key>) here to capture the ns of this file
    sel_tf_first = create_tf_selector_root(
        choice_key=_CHOICE_KEY,
        pose_key=_POSE_KEY,
        go_back_pose_key=_GO_BACK_POSE_KEY,
        is_first=True,
        fish_shoot_frame=fish_shoot_frame,
        shark_shoot_frame=shark_shoot_frame,
    )

    goto_target_first = goto.FromBlackboard(
        name="Go to first target",
        pose_key=_POSE_KEY,
        anchor_frame_name=TORPEDO_SHOOTER_TOP_FRAME,
    )

    fire_first = py_trees_ros.service_clients.FromConstant(
        name="Fire first torpedo",
        service_type=Trigger,
        service_name=ACTUATION_TOPIC_TOP,
        service_request=Trigger.Request(),
    )

    goto_back_centre = goto.FromBlackboard(
        name="Go back to centre",
        pose_key=_GO_BACK_POSE_KEY,
    )

    cluster_second = py_trees_ros.action_clients.FromConstant(
        name="Cluster the transforms before second shot",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=template_frame_optical,
            out_children=template_frame_optical_clustered,
            duration=CLUSTER_DURATION,
            use_cache=False,
        ),
    )

    sel_tf_second = create_tf_selector_root(
        choice_key=_CHOICE_KEY,
        pose_key=_POSE_KEY,
        go_back_pose_key=_GO_BACK_POSE_KEY,
        is_first=False,
        fish_shoot_frame=fish_shoot_frame,
        shark_shoot_frame=shark_shoot_frame,
    )

    goto_target_second = goto.FromBlackboard(
        name="Go to second target",
        pose_key=_POSE_KEY,
        anchor_frame_name=TORPEDO_SHOOTER_BOT_FRAME,
    )

    fire_second = py_trees_ros.service_clients.FromConstant(
        name="Fire second torpedo",
        service_type=Trigger,
        service_name=ACTUATION_TOPIC_BOT,
        service_request=Trigger.Request(),
    )

    srv_disable_detections = py_trees_ros.service_clients.FromConstant(
        name="Disable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(enabled=False),
        key_response=fk("torpedo_disable_detections"),
    )

    check_disable_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check disable succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("torpedo_disable_detections"),
            value=False,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

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

    seq_launch_torpedo.add_children(
        children=[
            srv_get_choice,
            srv_start_vision,
            check_start_vision_succeeded,
            cluster_board_centre,
            goto_torp_centre,
            stabilise_before_matching,
            srv_enable_detections,
            check_enable_succeeded,
            cluster_first,
            sel_tf_first,
            goto_target_first,
            py_trees.timers.Timer("Wait before fire", duration=STABILIZE_DURATION),
            fire_first,
            goto_back_centre,
            cluster_second,
            sel_tf_second,
            goto_target_second,
            py_trees.timers.Timer("Wait before fire", duration=STABILIZE_DURATION),
            fire_second,
            srv_disable_detections,
            check_disable_succeeded,
            srv_end_vision,
            check_end_vision_succeeded,
        ],
    )

    seq_torpedo_root.add_children(
        children=[
            py_trees.timers.Timer("Stabilise before task", duration=STABILIZE_DURATION),
            seq_launch_torpedo,
        ]
    )

    return seq_torpedo_root
