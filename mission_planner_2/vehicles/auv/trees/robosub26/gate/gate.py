import py_trees
from lifecycle_msgs.srv import ChangeState
from py_trees.decorators import Retry


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
    create_pose_clustering_goal,
    create_stamped_pose,
)
from mission_planner_2.vehicles.auv.config.node_registry import AUVSharedAction
from mission_planner_2.vehicles.auv.trees.goto import goto

######################### UPDATE CONSTANTS HERE #########################
VISION_SERVER_TOPIC = "/auv4/gate/manage_nodes"

GATE_APPROACH_HEIGHT = 0.5
FORWARD_DISTANCE = 1.5
NUM_RETRIES = 1

STABILISE_DURATION = 15

# CLUSTERING
CAMERA_FRAME = "auv4/front_cam_optical"
TEMPLATE_FRAME_YOLO = "gate/front"
TEMPLATE_FRAME_YOLO_CLUSTERED = "gate/clustered"
COLLECTION_DURATION = 15.0
SYNC_TOLERANCE = 0.1
MIN_POSES = 4
# STATIC TFs
GATE_CENTRE_FRAME = "gate/centre/view"
GATE_CENTRE_AFTER_GATE_FRAME = "gate/centre/after_gate"
GATE_LEFT_FRAME = "gate/left/view"
GATE_RIGHT_FRAME = "gate/right/view"


#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)
_START_VISION_KEY = fk("gate_start_vision")
_STOP_VISION_KEY = fk("gate_stop_vision")


def create_gate_root():
    # init
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

    force_success_start_vision = py_trees.decorators.FailureIsSuccess(
        name="Force Success Start Vision",
        child=retry_start_vision,
    )

    # cluster
    action_cluster_gate = shared_action_client.FromConstant(
        name="Cluster gate transforms",
        shared_action=AUVSharedAction.CLUSTER_POSE,
        action_goal=create_pose_clustering_goal(
            odom_topic="/auv4/nav/odom_ned",
            pose_stamped_topic="/auv4/gate/gate/front/pose",
            clustered_child_frame_id=TEMPLATE_FRAME_YOLO_CLUSTERED,
            collection_duration=COLLECTION_DURATION,
            sync_tolerance=SYNC_TOLERANCE,
            min_poses=MIN_POSES,
        ),
    )

    retry_cluster_gate = py_trees.decorators.Retry(
        name="Retry Cluster Gate",
        child=action_cluster_gate,
        num_failures=NUM_RETRIES,
    )

    # movement
    goto_gate_centre = goto.FromConstant(
        "Goto picture position",
        create_stamped_pose(GATE_CENTRE_FRAME),
        depth_override_value=GATE_APPROACH_HEIGHT,
        stabilize_duration=STABILISE_DURATION
    )

    goto_right_approach = goto.FromConstant(
        name="Goto right approach",
        pose=create_stamped_pose(GATE_RIGHT_FRAME),
        depth_override_value=GATE_APPROACH_HEIGHT,
        stabilize_duration=STABILISE_DURATION
    )

    forward_pose = create_stamped_pose("auv4/base_link_ned", position_x=FORWARD_DISTANCE)
    goto_through_gate = goto.FromConstant(
        name="Goto through gate",
        pose=forward_pose,
        depth_override_value=GATE_APPROACH_HEIGHT,
        stabilize_duration=STABILISE_DURATION
    )

    goto_centre_after_gate = goto.FromConstant(
        name="Goto centre after gate",
        pose=create_stamped_pose(GATE_CENTRE_AFTER_GATE_FRAME),
        yaw_threshold=0.1,
        stabilize_duration=STABILISE_DURATION
    )

    # cleanup
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
            force_success_start_vision,
            retry_cluster_gate,

            goto_gate_centre,
            goto_right_approach,
            goto_through_gate,
            goto_centre_after_gate,

            force_success_stop_vision,
        ]
    )

    return seq_gate_root
