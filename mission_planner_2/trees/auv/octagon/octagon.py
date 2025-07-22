import py_trees
import py_trees_ros
from bb_behavior_msgs.action import AlignAndCollect
from bb_perception_msgs.action import ClusterTf as ClusterTfAction
from bb_perception_msgs.srv import ClusterTf as ClusterTfSrv
from lifecycle_msgs.srv import ChangeState
from mission_planner_2.commons.blackboard import DynamicSetBlackboard
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
    create_clustering_request,
    create_stamped_pose,
)
from mission_planner_2.commons.tf_checker import create_tf_checker_from_constant_root
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.octagon.helpers import get_table_to_surface_target_yaw
from mission_planner_2.trees.auv.octagon.symbols import create_look_at_target_root
from mission_planner_2.trees.auv.octagon.trash import create_align_actuate_surface_root
from rclpy.qos import qos_profile_system_default
from std_srvs.srv import Trigger

# Generate namespace automatically from file path DONT set manually
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
VISION_SERVER_TOPIC = "/auv4/trash/manage_nodes"
ACTUATION_TOPIC = "/auv4/actuation/grabber"

CAMERA_FRAME = "auv4/front_cam_optical"
BASE_LINK_FRAME = "auv4/base_link_ned"

# Each of the following frames has a "/from_table", "/from_odom" and "/clustered" version
# NOTE: The base frame name does not exist.
BOTTLE_0_FRAME = "bottle_0"
BOTTLE_1_FRAME = "bottle_1"
LADLE_0_FRAME = "ladle_0"
LADLE_1_FRAME = "ladle_1"
PINK_BUCKET_FRAME = "pink_bucket"
YELLOW_BUCKET_FRAME = "yellow_bucket"

TABLE_CENTER_FRAME = "table/center"
TABLE_CENTER_FRAME_CLUSTERED = "table/center/clustered"

FISH_FRAME = "trash/fish"
SHARK_FRAME = "trash/shark"
FISH_FRAME_CLUSTERED = "trash/fish/clustered"
SHARK_FRAME_CLUSTERED = "trash/shark/clustered"
FISH_VIEW_FRAME = "trash/fish/clustered/view"
FISH_VIEW_FRAME_HARDCODED = "trash/fish/clustered/view/hardcoded"
SHARK_VIEW_FRAME = "trash/shark/clustered/view"
SHARK_VIEW_FRAME_HARDCODED = "trash/shark/clustered/view/hardcoded"

CLUSTER_DURATION = 4
NUM_ROTATIONS = 6
STABILIZE_DURATION = 5
WAIT_BETWEEN_ROTATIONS = 3
LOOK_AT_TARGET_PAUSE_DURATION = 4
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_CHOICE_KEY = fk("choice")
_TABLE_TO_SURFACE_TARGET_YAW_KEY = fk("go_surface_frame")
_START_VISION_KEY = fk("bin_start_vision")
_STOP_VISION_KEY = fk("bin_stop_vision")
_FISH_TF_KEY = fk("fish_tf")
_SHARK_TF_KEY = fk("shark_tf")
_TABLE_TF_KEY = fk("table_tf")
_LOOK_AT_TARGET_POSE_KEY = fk("look_at_target_pose")


def create_collection_root(trash_name: str, trash_frame: str, bucket_frame: str):
    """Picks up trash, surfaces, looks at the target, and drops it in the bucket."""
    seq_trash = py_trees.composites.Sequence(
        name=trash_name,
        memory=True,
    )
    seq_trash_pick_up = create_align_actuate_surface_root(
        trash_name,
        trash_frame,
        command=AlignAndCollect.Goal.CLOSE,
        cluster_duration=CLUSTER_DURATION,
        z_distance=0.20,
    )
    cluster_table_centre = py_trees_ros.action_clients.FromConstant(
        name="Cluster centre",
        action_type=ClusterTfAction,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=TABLE_CENTER_FRAME,
            out_children=TABLE_CENTER_FRAME_CLUSTERED,
            duration=CLUSTER_DURATION,
            use_cache=False,
        ),
    )
    # TODO: Rotate additional 90 degrees to table center to be able to see both buckets
    goto_table_centre = goto.FromConstant(
        name="Goto table centre",
        pose=create_stamped_pose(TABLE_CENTER_FRAME_CLUSTERED),
        ignore_depth=True,
    )
    # FIXME: Somehow need to stabilize or TF lookup for yaw gets the stale values while the
    # robot is turning to table center
    stabilize_before_drop = py_trees.timers.Timer(
        name="Stabilise before drop",
        duration=STABILIZE_DURATION,
    )
    seq_trash_drop = create_align_actuate_surface_root(
        trash_name,
        bucket_frame,
        command=AlignAndCollect.Goal.OPEN,
        depth_threshold=0.1,
        cluster_duration=CLUSTER_DURATION,
        z_distance=0.30,
    )
    seq_trash.add_children(
        children=[
            seq_trash_pick_up,
            cluster_table_centre,
            goto_table_centre,
            stabilize_before_drop,
            seq_trash_drop,
        ]
    )
    return seq_trash


def create_search_root():
    """Rotate 360 degrees and cluster the poses of the fish and shark tags. At the same time,
    cluster the pose of the table center."""
    in_children = [FISH_FRAME, SHARK_FRAME, TABLE_CENTER_FRAME]
    out_children = [
        FISH_FRAME_CLUSTERED,
        SHARK_FRAME_CLUSTERED,
        TABLE_CENTER_FRAME_CLUSTERED,
    ]

    seq_search = py_trees.composites.Sequence(name="Search", memory=True)
    cluster_node_start = py_trees_ros.service_clients.FromConstant(
        name="Cluster search",
        service_type=ClusterTfSrv,
        service_name="/auv4/cluster_tfs_srv",
        service_request=create_clustering_request(
            enabled=True,
            in_children=in_children,
            out_children=out_children,
            persistent=False,
        ),
    )
    goto_n_search_poses = goto.NFromConstant(
        name="Goto search poses",
        poses=[
            create_stamped_pose(
                frame_id=BASE_LINK_FRAME,
                yaw=360.0 / NUM_ROTATIONS,
            )
            for _ in range(NUM_ROTATIONS)
        ],
        anchor_frame_name=BASE_LINK_FRAME,
        specified_heading=True,
        wait_between_moves_sec=WAIT_BETWEEN_ROTATIONS,
    )
    cluster_node_stop = py_trees_ros.service_clients.FromConstant(
        name="Cluster search stop",
        service_type=ClusterTfSrv,
        service_name="/auv4/cluster_tfs_srv",
        service_request=create_clustering_request(
            enabled=False,
            in_children=in_children,
            out_children=out_children,
            persistent=False,
        ),
    )
    seq_search.add_children(
        [cluster_node_start, goto_n_search_poses, cluster_node_stop]
    )
    return seq_search


def create_octagon_root():
    """
    Create the root of the octagon tree.
    """
    root = py_trees.composites.Sequence(
        name="Octagon Root",
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
            operator=lambda x, y: x.success == y,
        ),
    )
    srv_get_choice = py_trees_ros.service_clients.FromConstant(
        name="Get choice",
        service_name="/auv4/choice/get_is_fish",
        service_type=Trigger,
        service_request=Trigger.Request(),
        key_response=_CHOICE_KEY,
    )

    ################## SEARCH PART #################
    par_search = create_search_root()
    symbol_tf_checker = create_tf_checker_from_constant_root(
        start_frames=[FISH_FRAME_CLUSTERED, SHARK_FRAME_CLUSTERED],
        update_keys=[_FISH_TF_KEY, _SHARK_TF_KEY],
        end_frames=["world_ned", "world_ned"],
        fallback_val=[FISH_VIEW_FRAME_HARDCODED, SHARK_VIEW_FRAME_HARDCODED],
    )
    table_tf_to_blackboard = py_trees_ros.transforms.ToBlackboard(
        name="Table TF to Blackboard",
        variable_name=_TABLE_TF_KEY,
        target_frame=TABLE_CENTER_FRAME_CLUSTERED,
        source_frame="world_ned",
        qos_profile=qos_profile_system_default,
    )
    dynamic_set_surface_yaw = DynamicSetBlackboard(
        name="Set surface yaw",
        key=[_CHOICE_KEY, _FISH_TF_KEY, _SHARK_TF_KEY, _TABLE_TF_KEY],
        update_key=_TABLE_TO_SURFACE_TARGET_YAW_KEY,
        overwrite=True,
        func=lambda choice, fish_tf, shark_tf, table_tf: get_table_to_surface_target_yaw(
            choice, fish_tf, shark_tf, table_tf
        ),
    )
    look_at_target = create_look_at_target_root(
        table_center_frame_clustered=TABLE_CENTER_FRAME_CLUSTERED,
        table_to_surface_target_yaw_key=_TABLE_TO_SURFACE_TARGET_YAW_KEY,
        look_at_target_pose_key=_LOOK_AT_TARGET_POSE_KEY,
        pause_duration=LOOK_AT_TARGET_PAUSE_DURATION,
    )

    ################## BOTTLE PART #################
    seq_bottle_0 = create_collection_root(
        trash_name="Bottle 0",
        trash_frame=BOTTLE_0_FRAME,
        bucket_frame=PINK_BUCKET_FRAME,
    )
    seq_ladle_0 = create_collection_root(
        trash_name="Ladle 0",
        trash_frame=LADLE_0_FRAME,
        bucket_frame=YELLOW_BUCKET_FRAME,
    )

    ############### ROTATION PARTS ###############
    # TODO: check the tfs with the within_dist to the basket to count before rotating
    goto_rotations = goto.NFromConstant(
        name="Go to rotations",
        poses=[
            create_stamped_pose(frame_id=BASE_LINK_FRAME, yaw=360.0),
            create_stamped_pose(frame_id=BASE_LINK_FRAME, yaw=360.0),
            create_stamped_pose(frame_id=BASE_LINK_FRAME, yaw=360.0),
            create_stamped_pose(frame_id=BASE_LINK_FRAME, yaw=360.0),
        ],
        wait_between_moves_sec=WAIT_BETWEEN_ROTATIONS,
    )

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
            operator=lambda x, y: x.success == y,
        ),
    )

    root.add_children(
        children=[
            srv_get_choice,
            srv_start_vision,
            check_start_vision_succeeded,
            par_search,
            symbol_tf_checker,
            table_tf_to_blackboard,
            dynamic_set_surface_yaw,
            look_at_target,
            seq_bottle_0,
            seq_ladle_0,
            goto_rotations,
            srv_end_vision,
            check_end_vision_succeeded,
        ]
    )

    return root
