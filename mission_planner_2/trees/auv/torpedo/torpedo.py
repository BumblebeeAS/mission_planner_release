import operator

import py_trees
import py_trees_ros
from bb_msgs.srv import IMPoseEstimatorToggleTemplate
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String, UInt8

from mission_planner_2.commons.blackboard import (
    DynamicSetBlackboard,
    full_key_generator,
)
from mission_planner_2.commons.namespace_utils import generate_namespace
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto_node
from mission_planner_2.trees.auv.torpedo.move_to_task import create_move_to_task_root

# Generate namespace automatically from file path
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

# TODO: figure out the actual offset for the shark
TEMPLATE_1_OFFSET_SHARK = {
    "x": 0.3,
    "y": 0.0,
    "z": 0.6,
    "roll": 90.0,
    "pitch": 90.0,
    "yaw": 0.0,
}

# TODO: figure out the actual offset for the fish should just be the translation xyz diff
# now its the exact same as the shark confirm need to change
TEMPLATE_1_OFFSET_FISH = {
    "x": 0.3,
    "y": 0.0,
    "z": 0.6,
    "roll": 90.0,
    "pitch": 90.0,
    "yaw": 0.0,
}


def _make_selection(choice: str):
    """
    Function to set the offset based on choice.
    """
    if choice == "shark":
        offset = TEMPLATE_1_OFFSET_SHARK
    else:  # fish
        offset = TEMPLATE_1_OFFSET_FISH

    return create_stamped_pose(
        "Task04_Tagging_01_optical/clustered",
        position_x=offset["x"],
        position_y=offset["y"],
        position_z=offset["z"],
        roll=offset["roll"],
        pitch=offset["pitch"],
        yaw=offset["yaw"],
    )


def create_torpedo_root():
    """
    Create the root of the torpedo tree.
    """
    TOGGLE_TEMPLATE_TOPIC = "/auv4/image_matching/toggle_template"

    # to handle the order of the torpedo we will choose one of the offsets to use before swapping
    # for now we sub to choice topic: /auv4/choice but i think launch file is a better way to do it

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
    # 3- enable detections + align to target
    # 4 - launch torpedo
    # 5 - disable detections

    enable_detections = py_trees_ros.service_clients.FromConstant(
        name="Enable Detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(
            enabled=True, template_name="Task04_Tagging_01.png"
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

    # TODO: remove all these if the new logic for choice works
    # UKF version
    # hole_pose = create_stamped_pose(
    #     "auv4/torpedo", 0.3, 0.0, 0.6, 90.0, 90.0, 0.0
    # )

    # Unfiltered version
    # hole_pose = create_stamped_pose(
    #     "Task04_Tagging_01_optical", 0.3, 0.0, 0.6, 90.0, 90.0, 0.0
    # )

    # Unfiltered version clustered
    # hole_pose = create_stamped_pose(
    #     "Task04_Tagging_01_optical/clustered", 0.3, 0.0, 0.6, 90.0, 90.0, 0.0
    # )

    # For manual testing with dummy tfs (if you are too lazy to keep running image matching).
    # ros2 run tf2_ros static_transform_publisher -3.3 0 -0.9 0 0 1.57 world fake_det # usually the pose the detection gives
    # ros2 run tf2_ros static_transform_publisher 0.3 0 0.6 0 1.57 1.57 fake_det hole

    # align_pose = create_stamped_pose("hole")

    choice_sub = py_trees_ros.subscribers.ToBlackboard(
        name="Choice Sub",
        topic_name="/auv4/choice",
        topic_type=String,
        qos_profile=qos_profile_system_default,
        blackboard_variables={fk("choice"): "data"},
    )

    set_choice = DynamicSetBlackboard(
        name="Set Choice",
        key="choice",
        namespace=NAMESPACE,
        update_key="hole",
        overwrite=True,
        func=_make_selection,
    )

    align_to_target = goto_node.FromBlackboard(
        name="Align to Target",
        parent_namespace=NAMESPACE,
        pose_key="hole",
    )

    # align_to_target = goto_node.FromConstant(
    #     name="Align to Target",
    #     parent_namespace=NAMESPACE,
    #     pose=hole_pose,
    #     # anchor_frame_name="auv4/front_cam_optical",
    # )

    # TODO: for now this is only one torpedo
    # later we will have to add the logic for more torpedoes

    set_torp_actuation = py_trees.behaviours.SetBlackboardVariable(
        name="Set Torpedo Actuation",
        variable_name=fk("torpedo_actuation"),
        variable_value=UInt8(data=2),
        overwrite=True,
    )

    fire_torpedo = py_trees_ros.publishers.FromBlackboard(
        name="Fire Torpedo",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=fk("torpedo_actuation"),
    )

    launch_seq.add_children(
        children=[
            choice_sub,
            set_choice,
            enable_detections,
            enable_detections_succeeded,
            py_trees.timers.Timer("Wait for Match", duration=5),
            align_to_target,
            set_torp_actuation,
            fire_torpedo,
            disable_detections,
            disable_detections_succeeded,
        ],
    )

    root.add_children(
        children=[
            create_move_to_task_root(),
            py_trees.timers.Timer("Stabilise before Match", duration=5),
            launch_seq,
        ]
    )

    return root
