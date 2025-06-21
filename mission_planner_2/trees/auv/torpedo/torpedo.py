import operator

import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from bb_perception_msgs.srv import IMPoseEstimatorToggleTemplate
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger

from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_clustering_goal
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.torpedo.tf_selector import create_tf_selector_root

# Generate namespace automatically from file path DONT set manually
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
TOGGLE_TEMPLATE_TOPIC = "/auv4/front_cam/image_matching/toggle_template"
TEMPLATE_NAME = "Task04_Tagging_02.png"

CAMERA_FRAME = "auv4/front_cam_optical"
TEMPLATE_FRAME_OPTICAL = "Task04_Tagging_02_optical"
TEMPLATE_FRAME_OPTICAL_CLUSTERED = "torpedo_2/clustered"
TORPEDO_SHOOTER_TOP_FRAME = "auv4/torpedo_shooter_top"
TORPEDO_SHOOTER_BOT_FRAME = "auv4/torpedo_shooter_bot"

TOP_TORP_UINT = UInt8(data=2)
BTM_TORP_UINT = UInt8(data=4)

CLUSTER_DURATION = 10
STABILIZE_DURATION = 10
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_CHOICE_KEY = fk("choice")
_POSE_KEY = fk("pose")
_GO_BACK_POSE_KEY = fk("go_back_pose")


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

    seq_torpedo_root = py_trees.composites.Sequence(
        name="Torpedo root",
        memory=True,
    )

    seq_launch_torpedo = py_trees.composites.Sequence(
        name="Launch torpedo",
        memory=True,
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
            enabled=True, template_name=TEMPLATE_NAME
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

    set_torp_top = py_trees.behaviours.SetBlackboardVariable(
        name="Set top torpedo actuation",
        variable_name=fk("torpedo_actuation"),
        variable_value=TOP_TORP_UINT,
        overwrite=True,
    )

    cluster_first = py_trees_ros.action_clients.FromConstant(
        name="Cluster the transforms before first shot",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=TEMPLATE_FRAME_OPTICAL,
            out_children=TEMPLATE_FRAME_OPTICAL_CLUSTERED,
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
    )

    goto_target_first = goto.FromBlackboard(
        name="Go to first target",
        pose_key=_POSE_KEY,
        anchor_frame_name=TORPEDO_SHOOTER_TOP_FRAME,
    )

    pub_fire_first = py_trees_ros.publishers.FromBlackboard(
        name="Fire first torpedo",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=fk("torpedo_actuation"),
    )

    goto_back_centre = goto.FromBlackboard(
        name="Go back to centre",
        pose_key=_GO_BACK_POSE_KEY,
    )

    set_torp_bottom = py_trees.behaviours.SetBlackboardVariable(
        name="Set bottom torpedo actuation",
        variable_name=fk("torpedo_actuation"),
        variable_value=BTM_TORP_UINT,
        overwrite=True,
    )

    cluster_second = py_trees_ros.action_clients.FromConstant(
        name="Cluster the transforms before second shot",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=TEMPLATE_FRAME_OPTICAL,
            out_children=TEMPLATE_FRAME_OPTICAL_CLUSTERED,
            duration=CLUSTER_DURATION,
            use_cache=False,
        ),
    )

    sel_tf_second = create_tf_selector_root(
        choice_key=_CHOICE_KEY,
        pose_key=_POSE_KEY,
        go_back_pose_key=_GO_BACK_POSE_KEY,
        is_first=False,
    )

    goto_target_second = goto.FromBlackboard(
        name="Go to second target",
        pose_key=_POSE_KEY,
        anchor_frame_name=TORPEDO_SHOOTER_BOT_FRAME,
    )

    pub_fire_second = py_trees_ros.publishers.FromBlackboard(
        name="Fire second torpedo",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=fk("torpedo_actuation"),
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

    seq_launch_torpedo.add_children(
        children=[
            srv_get_choice,
            srv_enable_detections,
            check_enable_succeeded,
            set_torp_top,
            cluster_first,
            sel_tf_first,
            goto_target_first,
            py_trees.timers.Timer("Wait between firings", duration=STABILIZE_DURATION),
            pub_fire_first,
            goto_back_centre,
            set_torp_bottom,
            cluster_second,
            sel_tf_second,
            goto_target_second,
            py_trees.timers.Timer("Wait between firings", duration=STABILIZE_DURATION),
            pub_fire_second,
            srv_disable_detections,
            check_disable_succeeded,
        ],
    )

    seq_torpedo_root.add_children(
        children=[
            py_trees.timers.Timer("Stabilise before task", duration=STABILIZE_DURATION),
            seq_launch_torpedo,
        ]
    )

    return seq_torpedo_root
