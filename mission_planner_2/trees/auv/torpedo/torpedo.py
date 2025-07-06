import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from bb_perception_msgs.srv import IMPoseEstimatorToggleTemplate
from lifecycle_msgs.srv import ChangeState
from mission_planner_2.commons import checked_service
from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.cluster_goto import (
    create_goto_cluster_from_bb_root,
    create_goto_cluster_from_bb_tf_tf_root,
    create_goto_cluster_from_constant_tf_tf_root,
)
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
    within_threshold,
    within_threshold_dist,
)
from mission_planner_2.trees.auv.goto import goto
from std_srvs.srv import Trigger

# Generate namespace automatically from file path DONT set manually
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
SELECTED_TEMPLATE = 2  # MUST be 1 or 2

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

CLUSTER_DURATION = 10
REALIGN_CLUSTER_DURATION = 4
STABILIZE_DURATION = 10
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_CHOICE_KEY = fk("choice")
_POSE_KEY = fk("pose")
_POSE_FRAME_KEY = fk("pose_frame")
_START_VISION_KEY = fk("torp_start_vision")
_STOP_VISION_KEY = fk("torp_stop_vision")
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

    set_anchor_frame = py_trees.behaviours.SetBlackboardVariable(
        name="Set anchor frame",
        variable_name=_ANCHOR_FRAME_KEY,
        variable_value=TORPEDO_SHOOTER_LEFT_FRAME,
        overwrite=True,
    )

    dynamic_set_pose = DynamicSetBlackboard(
        name="select torpedo frame",
        key=_CHOICE_KEY,
        update_key=_POSE_KEY,
        overwrite=True,
        func=lambda choice: create_stamped_pose(
            fish_shoot_frame if choice.success else shark_shoot_frame
        ),
    )

    dynamic_set_frame = DynamicSetBlackboard(
        name="select torpedo frame",
        key=_CHOICE_KEY,
        update_key=_POSE_FRAME_KEY,
        overwrite=True,
        func=lambda choice: fish_shoot_frame if choice.success else shark_shoot_frame,
    )

    cluster_node_first = py_trees_ros.action_clients.FromConstant(
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

    cluster_node_check_first = py_trees_ros.action_clients.FromConstant(
        name="Cluster the transforms before first shot",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=template_frame_optical,
            out_children=template_frame_optical_clustered,
            duration=REALIGN_CLUSTER_DURATION,
            use_cache=False,
        ),
    )

    goto_target_first = goto.FromBlackboard(
        name="Go to first target",
        pose_key=_POSE_KEY,
        anchor_frame_name=TORPEDO_SHOOTER_LEFT_FRAME,
    )

    goto_cluster_first = py_trees.decorators.FailureIsSuccess(
        name="Cluster and goto first",
        child=create_goto_cluster_from_bb_root(
            cluster_node=cluster_node_first,
            cluster_node_check=cluster_node_check_first,
            goto_node=goto_target_first,
            distance_threshold=0.05,
            yaw_threshold=3.0,
            retries=3,
            anchor_frame_key=_ANCHOR_FRAME_KEY,
            goto_pose_frame_key=_POSE_FRAME_KEY,
            within_threshold=within_threshold,
            stabilization_duration=2.5,
        ),
        # child=create_goto_cluster_from_bb_tf_tf_root(
        #     cluster_node=cluster_node_first,
        #     cluster_node_check=cluster_node_check_first,
        #     goto_node=goto_target_first,
        #     distance_threshold=0.05,
        #     retries=3,
        #     tf_frame_key=_POSE_FRAME_KEY,
        #     within_threshold=within_threshold_dist,
        #     stabilization_duration=2.5,
        # ),
    )

    fire_first = py_trees_ros.service_clients.FromConstant(
        name="Fire first torpedo",
        service_type=Trigger,
        service_name=ACTUATION_TOPIC_LEFT,
        service_request=Trigger.Request(),
    )

    goto_back_centre = goto.FromConstant(
        name="Go back to centre",
        pose=create_stamped_pose(CENTRE_VIEW_FRAME),
        anchor_frame_name=CAMERA_FRAME,
    )

    set_anchor_frame_2 = py_trees.behaviours.SetBlackboardVariable(
        name="Set anchor frame",
        variable_name=_ANCHOR_FRAME_KEY,
        variable_value=TORPEDO_SHOOTER_RIGHT_FRAME,
        overwrite=True,
    )

    dynamic_set_pose_2 = DynamicSetBlackboard(
        name="select torpedo frame",
        key=_CHOICE_KEY,
        update_key=_POSE_KEY,
        overwrite=True,
        func=lambda choice: create_stamped_pose(
            shark_shoot_frame if choice.success else fish_shoot_frame
        ),
    )

    dynamic_set_frame_2 = DynamicSetBlackboard(
        name="select torpedo frame second",
        key=_CHOICE_KEY,
        update_key=_POSE_FRAME_KEY,
        overwrite=True,
        func=lambda choice: shark_shoot_frame if choice.success else fish_shoot_frame,
    )

    cluster_node_second = py_trees_ros.action_clients.FromConstant(
        name="Cluster transforms before second shot",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=template_frame_optical,
            out_children=template_frame_optical_clustered,
            duration=CLUSTER_DURATION,
            use_cache=False,
        ),
    )

    cluster_node_check_second = py_trees_ros.action_clients.FromConstant(
        name="Cluster the transforms before first second",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=template_frame_optical,
            out_children=template_frame_optical_clustered,
            duration=REALIGN_CLUSTER_DURATION,
            use_cache=False,
        ),
    )

    goto_target_second = goto.FromBlackboard(
        name="Go to second target",
        pose_key=_POSE_KEY,
        anchor_frame_name=TORPEDO_SHOOTER_RIGHT_FRAME,
    )

    goto_cluster_second = py_trees.decorators.FailureIsSuccess(
        name="Cluster and goto second",
        child=create_goto_cluster_from_bb_root(
            cluster_node=cluster_node_second,
            cluster_node_check=cluster_node_check_second,
            goto_node=goto_target_second,
            distance_threshold=0.05,
            yaw_threshold=5.0,
            retries=3,
            anchor_frame_key=_ANCHOR_FRAME_KEY,
            goto_pose_frame_key=_POSE_FRAME_KEY,
            within_threshold=within_threshold,
            stabilization_duration=2.5,
        ),
        # child=create_goto_cluster_from_bb_tf_tf_root(
        #     cluster_node=cluster_node_second,
        #     cluster_node_check=cluster_node_check_second,
        #     goto_node=goto_target_second,
        #     distance_threshold=0.05,
        #     retries=3,
        #     tf_frame_key=_POSE_FRAME_KEY,
        #     within_threshold=within_threshold_dist,
        #     stabilization_duration=2.5,
        # ),
    )

    fire_second = py_trees_ros.service_clients.FromConstant(
        name="Fire second torpedo",
        service_type=Trigger,
        service_name=ACTUATION_TOPIC_RIGHT,
        service_request=Trigger.Request(),
    )

    srv_disable_detections = checked_service.FromConstant(
        name="Disable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(enabled=False),
        key_response=fk("torpedo_disable_detections"),
        check_func=lambda x: x.new_state,  # check if the service call was successful
    )

    srv_end_vision = checked_service.FromConstant(
        name="End vision pipeline",
        service_name=VISION_SERVER_TOPIC,
        service_type=ChangeState,
        service_request=create_end_vision_req(),
        key_response=_STOP_VISION_KEY,
        check_func=lambda x: x.success,  # check if the service call was successful
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
            set_anchor_frame,
            dynamic_set_pose,
            dynamic_set_frame,
            goto_cluster_first,
            fire_first,
            goto_back_centre,
            set_anchor_frame_2,
            dynamic_set_pose_2,
            dynamic_set_frame_2,
            goto_cluster_second,
            fire_second,
            srv_disable_detections,
            srv_end_vision,
        ],
    )

    seq_torpedo_root.add_children(
        children=[
            py_trees.timers.Timer("Stabilise before task", duration=STABILIZE_DURATION),
            seq_launch_torpedo,
        ]
    )

    return seq_torpedo_root
