import operator

import py_trees
import py_trees_ros
from bb_msgs.srv import IMPoseEstimatorToggleTemplate
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger
from transforms3d.euler import quat2euler

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.torpedo.move_to_task import create_move_to_task_root

# Generate namespace automatically from file path DONT set manually
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

# empty so that when break can tell global wrong DONT set manually
DETECTION_FRAME = ""

######################### UPDATE CONSTANTS HERE #########################
DETECTION_FRAME_MAP = {  # TODO: set the actual frame ids here reference cfg/cfg.yaml
    ("shark", True): "shark_hole",
    ("shark", False): "fish_hole",
    ("fish", True): "fish_hole",
    ("fish", False): "shark_hole",
}

TOGGLE_TEMPLATE_TOPIC = "/auv4/image_matching/toggle_template"

TOP_TORP_UINT = UInt8(data=2)
BTM_TORP_UINT = UInt8(data=4)
#########################################################################


def _make_selection(choice: Trigger.Response, isFirst=True):
    """
    Function to set the offset based on choice.
    """
    global DETECTION_FRAME

    DETECTION_FRAME = DETECTION_FRAME_MAP[(choice.message, isFirst)]
    return create_stamped_pose(
        frame_id=DETECTION_FRAME,
    )


def _tf_to_stamped_pose(tf):
    """
    Convert a TF to a StampedPose.
    """
    # convert quat to roll, pitch, yaw
    roll, pitch, yaw = quat2euler(
        [
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ],
    )

    return create_stamped_pose(
        frame_id=tf.header.frame_id,
        position_x=tf.transform.translation.x,
        position_y=tf.transform.translation.y,
        position_z=tf.transform.translation.z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    )


def create_torpedo_root():
    """
    Create the root of the torpedo tree.
    """

    root = py_trees.composites.Sequence(
        name="Torpedo Root",
        memory=True,
    )

    launch_seq = py_trees.composites.Sequence(
        name="Launch Torpedo",
        memory=True,
    )

    # contains the logic for launching the torpedo
    # 1 - move to task - in progress
    # 2 - make choice
    # 3 - enable detections
    # 4 - save tf + align to target
    # 5 - launch torpedo1
    # 6 - go to old tf saved in 4 to reset position
    # 7 - align to target 2
    # 8 - launch torpedo2
    # 9 - disable detections

    enable_detections = py_trees_ros.service_clients.FromConstant(
        name="Enable Detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(
            enabled=True, template_name="Task04_Tagging_02.png"
        ),
        key_response=fk("torpedo_enable_detections"),
    )

    disable_detections = py_trees_ros.service_clients.FromConstant(
        name="Disable Detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(enabled=False),
        key_response=fk("torpedo_disable_detections"),
    )

    # check srv call succeeded from the BB
    enable_detections_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Enable Detections Succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("torpedo_enable_detections"),
            value=True,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    disable_detections_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Disable Detections Succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("torpedo_disable_detections"),
            value=False,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    # Unfiltered version clustered
    hole_pose = create_stamped_pose(
        # "advay_please_remove_this", 0.44 - 0.10, 0.00 + 0.15, 0.5, 90.0, 90.0, 0.0
        "advay_please_remove_this",
        0.44 - 0.08,
        0.00 + 0.07,
        0.4,
        90.0,
        90.0,
        0.0,
    )

    # For manual testing with dummy tfs (if you are too lazy to keep running image matching).
    # ros2 run tf2_ros static_transform_publisher -3.3 0 -0.9 0 0 1.57 world fake_det # usually the pose the detection gives
    # ros2 run tf2_ros static_transform_publisher 0.3 0 0.6 0 1.57 1.57 fake_det hole

    get_choice = py_trees_ros.service_clients.FromConstant(
        name="Get Choice",
        service_name="/auv4/choice",
        service_type=Trigger,
        service_request=Trigger.Request(),
        key_response=fk("choice"),
    )

    set_choice1 = DynamicSetBlackboard(
        name="Set Choice 1",
        key="choice",
        namespace=NAMESPACE,
        update_key="hole",  # arbitrary key to set the hole pose
        overwrite=True,
        func=lambda x: _make_selection(x, isFirst=True),
    )

    # temp for pool test use
    align_to_target_const = goto.FromConstant(
        name="Align to Target",
        parent_namespace=NAMESPACE,
        pose=hole_pose,
    )

    save_tf = py_trees_ros.transforms.ToBlackboard(
        name="Save TF",
        variable_name=fk("reset_tf"),
        target_frame=DETECTION_FRAME,  # TODO: check if the global will actually update here
        source_frame="auv4/base_link_ned",
        qos_profile=qos_profile_system_default,
    )

    reconstruct_pose = DynamicSetBlackboard(
        name="Reconstruct Pose",
        key="reset_tf",
        namespace=NAMESPACE,
        update_key="reset_pose",
        overwrite=True,
        func=_tf_to_stamped_pose,
    )

    reset_saved_tf = goto.FromBlackboard(
        name="Reset to Saved TF",
        parent_namespace=NAMESPACE,
        pose_key="reset_pose",
    )

    align_to_target1 = goto.FromBlackboard(
        name="Align to Target1",
        parent_namespace=NAMESPACE,
        pose_key="hole",
    )

    set_choice2 = DynamicSetBlackboard(
        name="Set Choice 2",
        key="choice",
        namespace=NAMESPACE,
        update_key="hole",
        overwrite=True,
        func=lambda x: _make_selection(x, isFirst=False),
    )

    align_to_target2 = goto.FromBlackboard(
        name="Align to Target2",
        parent_namespace=NAMESPACE,
        pose_key="hole",
    )

    set_torp_actuation_top = py_trees.behaviours.SetBlackboardVariable(
        name="Set Torpedo Actuation top",
        variable_name=fk("torpedo_actuation"),
        variable_value=TOP_TORP_UINT,
        overwrite=True,
    )

    set_torp_actuation_btm = py_trees.behaviours.SetBlackboardVariable(
        name="Set Torpedo Actuation btm",
        variable_name=fk("torpedo_actuation"),
        variable_value=BTM_TORP_UINT,
        overwrite=True,
    )

    fire_torpedo1 = py_trees_ros.publishers.FromBlackboard(
        name="Fire Torpedo 1",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=fk("torpedo_actuation"),
    )

    fire_torpedo2 = py_trees_ros.publishers.FromBlackboard(
        name="Fire Torpedo 2",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=fk("torpedo_actuation"),
    )

    launch_seq.add_children(
        children=[
            get_choice,
            set_choice1,
            enable_detections,
            enable_detections_succeeded,
            py_trees.timers.Timer("Wait for Match", duration=20.0),
            # align_to_target_const,
            save_tf,
            align_to_target1,
            set_torp_actuation_top,
            fire_torpedo1,
            py_trees.timers.Timer("Wait between Firings", duration=5),
            reconstruct_pose,
            reset_saved_tf,
            set_choice2,
            align_to_target2,
            set_torp_actuation_btm,
            fire_torpedo2,
            disable_detections,
            disable_detections_succeeded,
        ],
    )

    root.add_children(
        children=[
            create_move_to_task_root(),
            py_trees.timers.Timer("Stabilise before Match", duration=10.0),
            launch_seq,
        ]
    )

    return root
