import py_trees
from bb_perception_msgs.srv import TrashToggleFrame
from bb_uav_msgs.action import Actuation, Land, Takeoff
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
    within_threshold_xyz,
)
from mission_planner_2.vehicles.shared.trees.cluster_goto import (
    create_goto_cluster_from_constant_root,
)
from mission_planner_2.vehicles.uav2.config.node_registry import UAV2SharedAction
from mission_planner_2.vehicles.uav2.trees.goto import goto

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
VISION_SERVER_TOPIC = "/uav2/tins/manage_nodes"
TOGGLE_TRASH_FRAME_SERVICE = "/uav2/tins/toggle_frame_clustered"
NUM_RETRIES = 3

CLUSTER_DURATION = 5.0
CLUSTER_MIN_POSES = 10
CLUSTER_SYNC_TOLERANCE = 0.10

ODOM_TOPIC = "/uav2/odom_ned"
BASE_LINK_FRAME = "uav2/base_link_frd"
WORLD_FRAME = "odom_ned"
CAMERA_FRAME = "uav2/wide_cam_optical"

HELIPAD_TOPIC = "/uav2/tins/tins/helipad/pose"
HELIPAD_FRAME_CLUSTERED = "helipad/clustered"
HELIPAD_FRAME_VIEW = "helipad/view"

TIN_COLOUR = "blue"
TIN_TOPIC = f"/uav2/tins/{TIN_COLOUR}_tin_0/from_odom/pose"
TIN_FRAME_CLUSTERED = f"{TIN_COLOUR}_tin_0/from_odom/clustered"
TIN_FRAME_VIEW = f"{TIN_COLOUR}_tin_0/from_odom/clustered/view"
TIN_FRAME_PICKUP_VIEW = f"{TIN_COLOUR}_tin_0/from_odom/clustered/pickup_view"

ACTUATION_FRAME = "uav2/actuation"
CAMERA_FRAME = "uav2/wide_a"

DISTANCE_THRESHOLD_XYZ = 0.03

LAND_SLEEP_DURATION = 7.0
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_START_VISION_KEY = fk("helipad_start_vision")
_STOP_VISION_KEY = fk("helipad_stop_vision")
_SRV_SURFACE_DEPTH_KEY = fk("helipad_surface_depth")


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

    retry_start_vision = py_trees.decorators.Retry(
        name="Retry Start Vision",
        child=srv_start_vision,
        num_failures=NUM_RETRIES,
    )

    action_cluster_helipad = shared_action_client.FromConstant(
        name="Cluster helipad poses",
        shared_action=UAV2SharedAction.CLUSTER,
        action_goal=create_pose_clustering_goal(
            odom_topic=ODOM_TOPIC,
            pose_stamped_topic=HELIPAD_TOPIC,
            clustered_child_frame_id=HELIPAD_FRAME_CLUSTERED,
            collection_duration=CLUSTER_DURATION,
            sync_tolerance=CLUSTER_SYNC_TOLERANCE,
            min_poses=CLUSTER_MIN_POSES,
        ),
    )

    retry_cluster_helipad = py_trees.decorators.Retry(
        name="Retry Cluster Helipad",
        child=action_cluster_helipad,
        num_failures=NUM_RETRIES,
    )

    goto_helipad_view = goto.FromConstant(
        name="Goto controls pose",
        anchor_frame_name=CAMERA_FRAME,
        pose=create_stamped_pose(frame_id=HELIPAD_FRAME_VIEW),
    )

    srv_surface_depth = checked_service.FromConstant(
        name="Stabilize",
        service_type=TrashToggleFrame,
        service_name=TOGGLE_TRASH_FRAME_SERVICE,
        service_request=TrashToggleFrame.Request(
            trash_frame_clustered=HELIPAD_FRAME_CLUSTERED,
            enable=True,
        ),
        key_response=_SRV_SURFACE_DEPTH_KEY,
        check_func=lambda x: x.success,
    )

    cluster_node = shared_action_client.FromConstant(
        name="Cluster tins",
        shared_action=UAV2SharedAction.CLUSTER,
        action_goal=create_pose_clustering_goal(
            odom_topic=ODOM_TOPIC,
            pose_stamped_topic=TIN_TOPIC,
            clustered_child_frame_id=TIN_FRAME_CLUSTERED,
            collection_duration=CLUSTER_DURATION,
            sync_tolerance=CLUSTER_SYNC_TOLERANCE,
            min_poses=CLUSTER_MIN_POSES,
        ),
    )

    retry_cluster_tins = py_trees.decorators.Retry(
        name="Retry Cluster Tins",
        child=cluster_node,
        num_failures=NUM_RETRIES,
    )

    cluster_node_check = shared_action_client.FromConstant(
        name="Check Cluster Tins",
        shared_action=UAV2SharedAction.CLUSTER,
        action_goal=create_pose_clustering_goal(
            odom_topic=ODOM_TOPIC,
            pose_stamped_topic=TIN_TOPIC,
            clustered_child_frame_id=TIN_FRAME_CLUSTERED,
            collection_duration=CLUSTER_DURATION,
            sync_tolerance=CLUSTER_SYNC_TOLERANCE,
            min_poses=CLUSTER_MIN_POSES,
        ),
    )

    retry_cluster_tins_check = py_trees.decorators.Retry(
        name="Retry Cluster Tins Check",
        child=cluster_node_check,
        num_failures=NUM_RETRIES,
    )

    goto_tins_xy = goto.FromConstant(
        name="Goto tins view",
        anchor_frame_name=ACTUATION_FRAME,
        pose=create_stamped_pose(frame_id=TIN_FRAME_VIEW),
    )

    goto_cluster = create_goto_cluster_from_constant_root(
        cluster_node=retry_cluster_tins,
        cluster_node_check=retry_cluster_tins_check,
        goto_node=goto_tins_xy,
        start_frames=["uav2/base_link_frd"],
        retries=NUM_RETRIES,
        goto_pose_frame=TIN_FRAME_VIEW,
        within_threshold_list=[
            within_threshold_xyz(DISTANCE_THRESHOLD_XYZ),
        ],
        stabilization_duration=0.5,
    )
    force_success_goto_cluster = py_trees.decorators.FailureIsSuccess(
        name="Force success goto cluster",
        child=goto_cluster,
    )

    goto_tins = goto.FromConstant(
        name="Goto tins view",
        anchor_frame_name=ACTUATION_FRAME,
        pose=create_stamped_pose(frame_id=TIN_FRAME_PICKUP_VIEW),
    )

    land = shared_action_client.FromConstant(
        name="Land to collect tin",
        shared_action=UAV2SharedAction.LAND,
        action_goal=Land.Goal(timeout=20.0),
    )

    takeoff = shared_action_client.FromConstant(
        name="Takeoff after collecting tin",
        shared_action=UAV2SharedAction.TAKEOFF,
        action_goal=Takeoff.Goal(
            altitude=1.5,
            x_threshold=0.1,
            y_threshold=0.1,
            z_threshold=0.1,
        ),
    )

    drop_tins = shared_action_client.FromConstant(
        name="Drop tins",
        shared_action=UAV2SharedAction.ACTUATION,
        action_goal=Actuation.Goal(
            enable_actuation=True,
        ),
    )

    reset_actuation = shared_action_client.FromConstant(
        name="Reset actuation",
        shared_action=UAV2SharedAction.ACTUATION,
        action_goal=Actuation.Goal(
            enable_actuation=False,
        ),
    )

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

    force_success_stop_vision = py_trees.decorators.FailureIsSuccess(
        name="Force success stop vision",
        child=retry_end_vision,
    )

    seq_helipad_root.add_children(
        children=[
            retry_start_vision,
            retry_cluster_helipad,
            goto_helipad_view,
            srv_surface_depth,
            force_success_goto_cluster,
            goto_tins,
            land,
            takeoff,
            drop_tins,
            reset_actuation,
            force_success_stop_vision,
        ]
    )

    return seq_helipad_root
