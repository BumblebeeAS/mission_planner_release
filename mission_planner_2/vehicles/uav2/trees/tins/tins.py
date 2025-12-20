import py_trees
from lifecycle_msgs.srv import ChangeState
from py_trees.decorators import Retry

from mission_planner_2.common.core import (
    checked_service,
    shared_action_client,
)
from mission_planner_2.common.util.detection_utils import (
    create_end_vision_req,
    create_start_vision_req,
)
from mission_planner_2.common.util.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.common.util.pose_utils import create_clustering_goal
from mission_planner_2.vehicles.uav2.trees.goto import goto
from mission_planner_2.vehicles.uav2.config.node_registry import UAV2SharedAction

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
VISION_SERVER_TOPIC = "/uav2/tins/manage_nodes"

CLUSTERING_DURATION = 4
STABILIZE_DURATION = 3.0

FORWARD_DISTANCE = 3.0
NUM_RETRIES = 3

BASE_LINK_FRAME = "uav2/base_link_frd"
WORLD_FRAME = "odom_ned"
CAMERA_FRAME = "uav2/wide_cam_optical"
TEMPLATE_FRAME_YOLO = "helipad"
TEMPLATE_FRAME_YOLO_CLUSTERED = "helipad/clustered"
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_START_VISION_KEY = fk("helipad_start_vision")
_STOP_VISION_KEY = fk("helipad_stop_vision")


def create_helipad_root():
    """Creates the helipad mission behavior tree."""

    seq_helipad_root = py_trees.composites.Sequence(
        name="Helipad root",
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

    action_cluster_helipad = shared_action_client.FromConstant(
        name="Cluster helipad transforms",
        shared_action=UAV2SharedAction.CLUSTER,
        action_goal=create_clustering_goal(
            in_children=TEMPLATE_FRAME_YOLO,
            out_children=TEMPLATE_FRAME_YOLO_CLUSTERED,
            out_parents=WORLD_FRAME,
            duration=CLUSTERING_DURATION,
            use_cache=False,
        ),
    )

    retry_cluster_helipad = py_trees.decorators.Retry(
        name="Retry Cluster Helipad",
        child=action_cluster_helipad,
        num_failures=NUM_RETRIES,
    )

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
        name="Force success stop vision",
        child=retry_end_vision,
    )

    # Assemble tree in execution order
    seq_helipad_root.add_children(
        children=[
            retry_start_vision,
            retry_cluster_helipad,
            force_success_stop_vision,
        ]
    )

    return seq_helipad_root
