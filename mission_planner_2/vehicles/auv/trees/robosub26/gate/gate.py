import operator

import py_trees
import py_trees_ros
from bb_behavior_msgs.action import ControlledSpin
from bb_controls_msgs.srv import Controller
from lifecycle_msgs.srv import ChangeState
from py_trees.decorators import Retry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from mission_planner_2.common.core import checked_service, shared_action_client
from mission_planner_2.common.util.detection_utils import (
    create_end_vision_req,
    create_start_vision_req,
)
from mission_planner_2.common.util.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.common.util.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
)
from mission_planner_2.vehicles.auv.config.node_registry import AUVSharedAction
from mission_planner_2.vehicles.auv.trees.goto import goto
from mission_planner_2.vehicles.shared.trees.tf_checker import (
    create_tf_checker_from_constant_root,
)

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
VISION_SERVER_TOPIC = "/auv4/gate/manage_nodes"
CONTROLS_SRV_TOPIC = "/auv4/controls/controller"

CLUSTERING_DURATION = 10
STABILIZE_DURATION = 6.0

FORWARD_DISTANCE = 3.0
NUM_RETRIES = 1

BASE_LINK_FRAME = "auv4/base_link_ned"
WORLD_FRAME = "world_ned"
CAMERA_FRAME = "auv4/front_cam_optical"
TEMPLATE_FRAME_YOLO = "gate/front"
TEMPLATE_FRAME_YOLO_CLUSTERED = "gate/clustered"
GATE_CENTRE_FRAME = "gate/centre/view"
GATE_LEFT_FRAME = "gate/left/view"
GATE_RIGHT_FRAME = "gate/right/view"

GATE_DEPTH = 1.3

GATE_ORIENTATION_TOPIC = "/auv4/gate/shark_fish"
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_GATE_ORIENTATION_KEY = fk("orientation")
_IS_LEFT_KEY = "/global/is_left_side"  # Global key for left option or not
_START_VISION_KEY = fk("gate_start_vision")
_STOP_VISION_KEY = fk("gate_stop_vision")
_YAW_BEFORE_GATE = "/global/yaw_before_gate"


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

    srv_start_vision = checked_service.FromConstant(
        name="Start vision",
        service_type=ChangeState,
        service_name=VISION_SERVER_TOPIC,
        service_request=create_start_vision_req(),
        key_response=_START_VISION_KEY,
        check_func=lambda x: x.success,
    )

    retry_start_vision = Retry(
        name="Retry Start Vision",
        child=srv_start_vision,
        num_failures=NUM_RETRIES,
    )

    # Step 3: Cluster gate transforms
    action_cluster_gate = shared_action_client.FromConstant(
        name="Cluster gate transforms",
        shared_action=AUVSharedAction.CLUSTER,
        action_goal=create_clustering_goal(
            in_children=TEMPLATE_FRAME_YOLO,
            out_children=TEMPLATE_FRAME_YOLO_CLUSTERED,
            duration=CLUSTERING_DURATION,
            use_cache=False,
        ),
    )

    retry_cluster_gate = py_trees.decorators.Retry(
        name="Retry Cluster Gate",
        child=action_cluster_gate,
        num_failures=NUM_RETRIES,
    )

    # Step 5: Move to picture position
    goto_gate_centre = goto.FromConstant(
        "Goto picture position",
        create_stamped_pose(GATE_CENTRE_FRAME),
        depth_override_value=GATE_DEPTH,
    )

    get_yaw_before_gate = create_tf_checker_from_constant_root(
        start_frames=["world_ned"],
        end_frames=["auv4/base_link_ned"],
        update_keys=[_YAW_BEFORE_GATE],
        timeout=120.0,
        fallback_val=[None],
    )

    goto_right_approach = goto.FromConstant(
        name="Goto right approach",
        pose=create_stamped_pose(GATE_RIGHT_FRAME),
        depth_override_value=GATE_DEPTH,
    )

    # Step 11: Pass through gate
    forward_pose = create_stamped_pose(GATE_RIGHT_FRAME, position_x=FORWARD_DISTANCE)
    goto_through_gate = goto.FromConstant(
        name="Goto through gate",
        pose=forward_pose,
        depth_override_value=GATE_DEPTH,
    )

    goto_after_gate = goto.FromConstant(
        name="Goto centre after gate",
        pose=create_stamped_pose("gate/centre/after_gate"),
        depth_override_value=GATE_DEPTH,
    )

    goto_pole = goto.FromConstant(
        name="Goto imaginary pole",
        pose=create_stamped_pose("gate/centre/after_gate", position_x=2.0),
        depth_override_value=GATE_DEPTH,
    )

    goto_back_to_gate = goto.FromConstant(
        name="Goto centre after gate",
        pose=create_stamped_pose("gate/centre/after_gate", yaw=180.0),
        depth_override_value=GATE_DEPTH,
    )

    goto_return_right_approach = goto.FromConstant(
        name="Goto returning right approach",
        pose=create_stamped_pose(
            GATE_LEFT_FRAME, position_x=FORWARD_DISTANCE, yaw=180.0
        ),
        depth_override_value=GATE_DEPTH,
    )

    goto_return_through_gate = goto.FromConstant(
        name="finish", pose=create_stamped_pose(GATE_LEFT_FRAME, yaw=180.0)
    )

    # Start of spinning
    seq_yaw_spin = py_trees.composites.Sequence(name="yaw_spin", memory=True)

    srv_disable_controls = checked_service.FromConstant(
        name="Disable controls (for spin)",
        service_name=CONTROLS_SRV_TOPIC,
        service_type=Controller,
        service_request=Controller.Request(
            enable=False,
            pause=False,
            disable_altitude=False,
        ),
    )

    spin = shared_action_client.FromConstant(
        name="Call controlled spin",
        shared_action=AUVSharedAction.CONTROLLED_SPIN,
        action_goal=ControlledSpin.Goal(
            yaw_amount=180.0,
            yaw_tolerance=3.0,
            yaw_rate=20.0,
            timeout_seconds=30.0,
        ),
    )

    force_succeed_spin = py_trees.decorators.FailureIsSuccess(
        name="Force success spin",
        child=spin,
    )

    srv_enable_controls = checked_service.FromConstant(
        name="Enable controls (for spin)",
        service_name=CONTROLS_SRV_TOPIC,
        service_type=Controller,
        service_request=Controller.Request(
            enable=True,
            pause=False,
            disable_altitude=False,
        ),
    )

    seq_yaw_spin.add_children(
        children=[srv_disable_controls, force_succeed_spin, srv_enable_controls]
    )
    # End of spinning

    srv_end_vision = checked_service.FromConstant(
        name="End vision",
        service_type=ChangeState,
        service_name=VISION_SERVER_TOPIC,
        service_request=create_end_vision_req(),
        key_response=_STOP_VISION_KEY,
        check_func=lambda x: x.success,
    )

    retry_end_vision = Retry(
        name="Retry End Vision",
        child=srv_end_vision,
        num_failures=NUM_RETRIES,
    )

    force_success_stop_vision = py_trees.decorators.FailureIsSuccess(
        name="Force success stop vision", child=retry_end_vision
    )

    # Assemble tree in execution order
    seq_gate_root.add_children(
        children=[
            # goto_towards_gate,
            retry_start_vision,
            retry_cluster_gate,
            goto_gate_centre,
            goto_right_approach,
            # get_yaw_before_gate,
            goto_through_gate,
            goto_pole,
            seq_yaw_spin,
            goto_back_to_gate,
            goto_return_right_approach,
            goto_return_through_gate,
            force_success_stop_vision,
        ]
    )

    return seq_gate_root
