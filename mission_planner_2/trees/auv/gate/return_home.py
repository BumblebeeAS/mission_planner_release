import py_trees
from lifecycle_msgs.srv import ChangeState

from mission_planner_2.commons import checked_service, shared_action_client
from mission_planner_2.commons.detection_utils import (
    create_end_vision_req,
    create_start_vision_req,
)
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.node_registry import SharedAction
from mission_planner_2.commons.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
)
from mission_planner_2.trees.auv.goto import goto

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
VISION_SERVER_TOPIC = "/auv4/gate_back/manage_nodes"

CLUSTERING_DURATION = 15
STABILIZE_DURATION = 5.0

GATE_APPROACH_HEIGHT = 0.40
FORWARD_DISTANCE = 3.0
NUM_RETRIES = 3

WORLD_FRAME = "world_ned"
CAMERA_FRAME = "auv4/front_cam_optical"
TEMPLATE_FRAME_YOLO = "gate"
TEMPLATE_FRAME_YOLO_CLUSTERED = "gate/clustered"
GATE_CENTRE_FRAME = "gate/centre"
#########################################################################


def create_return_root():

    seq_return_root = py_trees.composites.Sequence(
        name="Return root",
        memory=True,
    )

    srv_start_vision = checked_service.FromConstant(
        name="Start vision",
        service_type=ChangeState,
        service_name=VISION_SERVER_TOPIC,
        service_request=create_start_vision_req(),
        key_response=fk("gate_start_vision"),
        check_func=lambda x: x.success,
    )

    retry_start_vision = py_trees.decorators.Retry(
        name="Retry Start Vision",
        child=srv_start_vision,
        num_failures=NUM_RETRIES,
    )

    # TODO: Eventually, we should cache some position after passing through the gate
    #       and return to this position after doing all the tasks
    gate_init_pose = create_stamped_pose("world_ned", position_z=GATE_APPROACH_HEIGHT)
    goto_after_gate = goto.FromConstant("Goto after gate", gate_init_pose)

    action_cluster_gate = shared_action_client.FromConstant(
        name="Cluster gate transforms",
        shared_action=SharedAction.CLUSTER,
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

    goto_after_gate_center = goto.FromConstant(
        "Goto after gate centre", create_stamped_pose(GATE_CENTRE_FRAME)
    )

    forward_pose = create_stamped_pose(
        "auv4/base_link_ned", position_x=FORWARD_DISTANCE
    )
    goto_through_gate = goto.FromConstant("Goto through gate", forward_pose)

    srv_end_vision = checked_service.FromConstant(
        name="End vision",
        service_type=ChangeState,
        service_name=VISION_SERVER_TOPIC,
        service_request=create_end_vision_req(),
        key_response=fk("gate_end_vision"),
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
            retry_start_vision,
            goto_after_gate,
            retry_cluster_gate,
            goto_after_gate_center,
            goto_through_gate,
            force_success_end_vision,
        ]
    )

    return seq_return_root
