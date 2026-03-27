import py_trees
from lifecycle_msgs.srv import ChangeState

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

GATE_APPROACH_HEIGHT = 0.40
FORWARD_DISTANCE = 3.0
NUM_RETRIES = 3

# CLUSTERING
ODOM_TOPIC = "/auv4/nav/odom_ned"
TEMPLATE_POSE_YOLO = "/auv4/gate/gate/back/pose"
TEMPLATE_FRAME_YOLO_CLUSTERED = "gate/clustered"
COLLECTION_DURATION = 4.0
SYNC_TOLERANCE = 0.1
MIN_POSES = 4

# STATIC TFs
GATE_CENTRE_FRAME = "gate/centre/after_gate"
GATE_LEFT_FRAME = "gate/left/view"
GATE_RIGHT_FRAME = "gate/right/view"
#########################################################################
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)
_START_VISION_KEY = fk("gate_start_vision")
_STOP_VISION_KEY = fk("gate_stop_vision")


def create_return_root():

    # init
    seq_return_root = py_trees.composites.Sequence(
        name="Return root",
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

    retry_start_vision = py_trees.decorators.Retry(
        name="Retry Start Vision",
        child=srv_start_vision,
        num_failures=NUM_RETRIES,
    )

    force_success_start_vision = py_trees.decorators.FailureIsSuccess(
        name="Force Success Start Vision",
        child=retry_start_vision,
    )

    # assumes gate alr visible
    action_cluster_gate = shared_action_client.FromConstant(
        name="Cluster gate transforms",
        shared_action=AUVSharedAction.CLUSTER_POSE,
        action_goal=create_pose_clustering_goal(
            odom_topic=ODOM_TOPIC,
            pose_stamped_topic=TEMPLATE_POSE_YOLO,
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



    # movements
    goto_gate_left = goto.FromConstant(
        name="Goto gate left",
        pose=create_stamped_pose(GATE_LEFT_FRAME),
        depth_override_value=GATE_APPROACH_HEIGHT,
        stabilize_duration=15
    )

    goto_through_gate = goto.FromConstant(
        name="Goto through the gate",
        pose= create_stamped_pose("auv4/base_link_ned", position_x=FORWARD_DISTANCE),
        depth_override_value=GATE_APPROACH_HEIGHT,
        stabilize_duration=15
    )

    # clean up
    srv_end_vision = checked_service.FromConstant(
        name="End vision",
        service_type=ChangeState,
        service_name=VISION_SERVER_TOPIC,
        service_request=create_end_vision_req(),
        key_response=_STOP_VISION_KEY,
        check_func=lambda x: x.success,
    )

    retry_end_vision = py_trees.decorators.Retry(
        name="Retry End Vision",
        child=srv_end_vision,
        num_failures=NUM_RETRIES,
    )

    force_success_end_vision = py_trees.decorators.FailureIsSuccess(
        name="Force Success End Vision",
        child=retry_end_vision,
    )

    seq_return_root.add_children(
        children=[
            # init
            force_success_start_vision,
            retry_cluster_gate,
            # movements
            goto_gate_left,
            goto_through_gate,
            # cleanup
            force_success_end_vision,
        ]
    )

    return seq_return_root
