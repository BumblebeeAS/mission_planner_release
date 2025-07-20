import operator

import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from lifecycle_msgs.srv import ChangeState
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
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

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
VISION_SERVER_TOPIC = "/auv4/gate_front/manage_nodes"

CLUSTERING_DURATION = 4
STABILIZE_DURATION = 7.0

FORWARD_DISTANCE = 3.0

CAMERA_FRAME = "auv4/front_cam_optical"
TEMPLATE_FRAME_YOLO = "gate"
TEMPLATE_FRAME_YOLO_CLUSTERED = "gate/clustered"
GATE_CENTRE_FRAME = "gate/centre/view"
GATE_LEFT_FRAME = "gate/left/view"
GATE_RIGHT_FRAME = "gate/right/view"

GATE_ORIENTATION_TOPIC = "/auv4/gate/shark_fish"
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_CHOICE_KEY = fk("choice")
_GATE_ORIENTATION_KEY = fk("orientation")
_GATE_LEFT_POSE_KEY = fk("gate_left_pose")
_GATE_RIGHT_POSE_KEY = fk("gate_right_pose")
_IS_LEFT_KEY = "/global/is_left_side"  # Global key for left option or not
_START_VISION_KEY = fk("gate_start_vision")
_STOP_VISION_KEY = fk("gate_stop_vision")


def create_gate_root():
    """
    For sim.

    gate_init_pose = create_stamped_pose("world_ned", 5.98, 2.48, 1.16, 0.0, 0.0, -90)

    Publish the following tf to mock the gate detection:
    ros2 run tf2_ros static_transform_publisher 7 0 1.5 -1.57 0 0 world_ned auv4/gate
    """

    # Step 1: Root sequence
    seq_gate_root = py_trees.composites.Sequence(
        name="Gate root",
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
            operator=lambda x, y: operator.eq(x.success, y),
        ),
    )

    # Step 3: Cluster gate transforms
    action_cluster_gate = py_trees_ros.action_clients.FromConstant(
        name="Cluster gate transforms",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=TEMPLATE_FRAME_YOLO,
            out_children=TEMPLATE_FRAME_YOLO_CLUSTERED,
            duration=CLUSTERING_DURATION,
            use_cache=False,
        ),
    )

    # Step 5: Move to picture position
    goto_see_pictures = goto.FromConstant(
        "Goto picture position", create_stamped_pose(GATE_CENTRE_FRAME)
    )

    # Step 6: Wait to stabilize
    timer_stabilize_main = py_trees.timers.Timer(
        "Stabilize before task", STABILIZE_DURATION
    )

    # Step 7: Get fish choice
    srv_get_fish_choice = py_trees_ros.service_clients.FromConstant(
        name="Get fish choice",
        service_type=Trigger,
        service_name="/auv4/choice/get_is_fish",
        service_request=Trigger.Request(),
        key_response=_CHOICE_KEY,
    )

    # Step 8: Get gate orientation
    sub_gate_orientation = py_trees_ros.subscribers.ToBlackboard(
        name="Get shark fish orientation",
        topic_name=GATE_ORIENTATION_TOPIC,
        topic_type=String,
        qos_profile=qos_profile_sensor_data,
        blackboard_variables={_GATE_ORIENTATION_KEY: None},
    )

    # Step 9: Select gate side (selector with left side attempt)
    sel_gate_side = py_trees.composites.Selector(name="Select gate side", memory=True)

    seq_try_left_side = py_trees.composites.Sequence(name="Try left side", memory=True)

    # TODO: change to xor func shorten this
    check_is_left = py_trees.behaviours.CheckBlackboardVariableValues(
        name="Check if left side",
        checks=[
            py_trees.common.ComparisonExpression(
                variable=_CHOICE_KEY,
                value=True,
                operator=lambda x, y: operator.eq(x.success, y),
            ),
            py_trees.common.ComparisonExpression(
                variable=_GATE_ORIENTATION_KEY,
                value="fish_shark",
                operator=lambda x, y: operator.eq(x.data, y),
            ),
        ],
        operator=operator.eq,
    )

    write_is_left = py_trees.behaviours.SetBlackboardVariable(
        name="Write is left side",
        variable_name=_IS_LEFT_KEY,
        variable_value=True,
        overwrite=True,
    )

    goto_left_approach = goto.FromConstant(
        name="Goto left approach",
        pose=create_stamped_pose(GATE_LEFT_FRAME),
    )

    seq_go_right_side = py_trees.composites.Sequence(name="Go right side", memory=True)

    write_not_is_left = py_trees.behaviours.SetBlackboardVariable(
        name="Write is left side",
        variable_name=_IS_LEFT_KEY,
        variable_value=False,
        overwrite=True,
    )

    goto_right_approach = goto.FromConstant(
        name="Goto right approach",
        pose=create_stamped_pose(GATE_RIGHT_FRAME),
    )

    seq_go_right_side.add_children(children=[write_not_is_left, goto_right_approach])

    seq_try_left_side.add_children(
        children=[check_is_left, write_is_left, goto_left_approach]
    )
    sel_gate_side.add_children(children=[seq_try_left_side, seq_go_right_side])

    # Step 10: Wait to stabilize before passage
    timer_stabilize_final = py_trees.timers.Timer(
        "Stabilize before passage", STABILIZE_DURATION
    )

    # Step 11: Pass through gate
    forward_pose = create_stamped_pose(
        "auv4/base_link_ned", position_x=FORWARD_DISTANCE
    )
    goto_through_gate = goto.FromConstant("Goto through gate", forward_pose)

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
    # Assemble tree in execution order
    seq_gate_root.add_children(
        children=[
            # goto_towards_gate,
            srv_start_vision,
            check_start_vision_succeeded,
            action_cluster_gate,
            goto_see_pictures,
            timer_stabilize_main,
            srv_get_fish_choice,
            sub_gate_orientation,
            sel_gate_side,
            timer_stabilize_final,
            goto_through_gate,
            srv_end_vision,
            check_end_vision_succeeded,
        ]
    )

    return seq_gate_root
