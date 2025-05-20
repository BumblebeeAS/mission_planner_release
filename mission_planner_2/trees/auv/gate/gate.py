import operator

import py_trees
from bb_msgs.srv import IMPoseEstimatorToggleTemplate

from mission_planner_2.commons import service_clients
from mission_planner_2.commons.blackboard import full_key_generator
from mission_planner_2.commons.namespace_utils import generate_namespace
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto_node

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)


def _gen_enable_req():
    """
    Generate the enable service request to enable gate detections.
    """
    # TODO: This may not be the actual detection enable request
    req = IMPoseEstimatorToggleTemplate.Request()
    req.enabled = True
    req.template_name = "gate"
    return req


def _gen_disable_req():
    """
    Generate the disable service request to disable gate detections.
    """
    req = IMPoseEstimatorToggleTemplate.Request()
    req.enabled = False
    return req


def create_gate_root():
    """
    Create the root of the gate tree.

    Execution Flow:
    1. Move closer to the gate
    2. Enable detections
    3. Check if enable succeeded
    4. Move to gate pose
    5. Pass through gate
    6. Disable detections
    7. Disable detections succeeded
    """

    TOGGLE_DETECTIONS_TOPIC = "/auv4/image_matching/toggle_template"

    root = py_trees.composites.Sequence(
        name="Gate Root",
        memory=True,
    )

    enable_detections = service_clients.FromConstant(
        name="Enable Detections",
        namespace=NAMESPACE,
        service_name=TOGGLE_DETECTIONS_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=_gen_enable_req(),
        key_response="gate_enable_detections",
    )

    disable_detections = service_clients.FromConstant(
        name="Disable Detections",
        namespace=NAMESPACE,
        service_name=TOGGLE_DETECTIONS_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=_gen_disable_req(),
        key_response="gate_disable_detections",
    )

    # check srv call succeeded from the BB
    enable_detections_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Enable Detections Succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("gate_enable_detections"),
            value=True,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    disable_detections_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Disable Detections Succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=fk("gate_disable_detections"),
            value=True,
            operator=lambda x, y: operator.eq(x.new_state, y),
        ),
    )

    gate_init_pose = create_stamped_pose(
        frame_id="world_ned",
        position_x=6.0,
        position_y=2.0,
        position_z=1.5,
        roll=0.0,
        pitch=0.0,
        yaw=-90.0,
    )

    gate_target_pose = create_stamped_pose("auv4/gate")
    # Publish the following tf to mock the gate detection
    # ros2 run tf2_ros static_transform_publisher 7 0 1.5 -1.57 0 0 world_ned auv4/gate

    gate_passthrough_pose = create_stamped_pose(
        frame_id="auv4/base_link",
        position_x=2.0,
        position_y=0.0,
        position_z=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
    )

    move_towards_gate = goto_node.FromConstant("move_towards_gate", NAMESPACE, gate_init_pose)

    move_to_gate_target = goto_node.FromConstant(
        "move_to_gate_target",
        NAMESPACE,
        gate_target_pose,
    )

    pass_through_gate = goto_node.FromConstant(
        "pass_through_gate", NAMESPACE, gate_passthrough_pose
    )

    root.add_children(
        children=[
            move_towards_gate,
            py_trees.timers.Timer(
                name="Wait for 5 seconds",
                duration=0.5,
            ),
            # enable_detections,
            # enable_detections_succeeded,
            move_to_gate_target,
            py_trees.timers.Timer(
                name="Wait for 5 seconds",
                duration=0.5,
            ),
            pass_through_gate,
            py_trees.timers.Timer(
                name="Wait for 5 seconds",
                duration=0.5,
            ),
            # disable_detections,
            # disable_detections_succeeded,
        ]
    )

    return root
