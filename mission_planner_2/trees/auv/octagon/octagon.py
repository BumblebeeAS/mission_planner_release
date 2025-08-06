import py_trees
import py_trees_ros
from bb_behavior_msgs.action import AlignAndCollect
from bb_perception_msgs.srv import ClusterTfSrv
from lifecycle_msgs.srv import ChangeState

from mission_planner_2.commons import shared_action_client
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
    create_clustering_request,
    create_stamped_pose,
)
from mission_planner_2.commons.search import create_search_bot_layered_square_root
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.octagon.symbols import create_look_at_target_root
from mission_planner_2.trees.auv.octagon.trash import (
    create_align_actuate_surface_root,
    create_checked_collection_root,
    create_spin_root,
)

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
BOTTLE_FRAME = "bottle_0"
LADLE_FRAME = "ladle_0"
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
NUM_SQUARES = 1
OFFSET_COEFF = 1.0
DROP_Z_DISTANCE = 0.40
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_CHOICE_KEY = "/global/choice_is_fish"
_START_VISION_KEY = fk("bin_start_vision")
_STOP_VISION_KEY = fk("bin_stop_vision")


def create_goto_table_centre_root():
    root = py_trees.composites.Sequence(
        name="Goto table centre",
        memory=True,
    )

    cluster_table_centre = shared_action_client.FromConstant(
        name="Cluster centre",
        shared_action=SharedAction.CLUSTER,
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

    root.add_children(
        [
            cluster_table_centre,
            goto_table_centre,
        ]
    )

    return root


def create_collection_root(trash_name: str, trash_frame: str, bucket_frame: str):
    """Picks up trash, surfaces, looks at the target, and drops it in the bucket."""
    seq_trash = py_trees.composites.Sequence(
        name=trash_name,
        memory=True,
    )

    goto_table_centre_pick_up = create_goto_table_centre_root()

    seq_trash_pick_up = create_align_actuate_surface_root(
        trash_name,
        trash_frame,
        command=AlignAndCollect.Goal.CLOSE,
        depth_rate=0.05,
        cluster_duration=CLUSTER_DURATION,
    )

    goto_table_centre_drop = create_goto_table_centre_root()

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
        cluster_duration=CLUSTER_DURATION,
        z_distance=DROP_Z_DISTANCE,
    )
    seq_trash.add_children(
        children=[
            goto_table_centre_pick_up,
            seq_trash_pick_up,
            goto_table_centre_drop,
            # stabilize_before_drop,
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
        [
            cluster_node_start,
            goto_n_search_poses,
            cluster_node_stop,
        ]
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

    ################## SEARCH PART #################
    par_search = create_search_root()

    look_at_target = create_look_at_target_root(
        choice_key=_CHOICE_KEY,
        fish_frame_clustered=FISH_FRAME_CLUSTERED,
        shark_frame_clustered=SHARK_FRAME_CLUSTERED,
        table_center_frame_clustered=TABLE_CENTER_FRAME_CLUSTERED,
        pause_duration=LOOK_AT_TARGET_PAUSE_DURATION,
    )

    seq_search = create_search_bot_layered_square_root(
        fwd=1.0,
        back=0.3,
        left=0.5,
        right=0.5,
        num_squares=NUM_SQUARES,
        object_frame=TABLE_CENTER_FRAME,
        object_frame_clustered=TABLE_CENTER_FRAME_CLUSTERED,
        offset_coeff=OFFSET_COEFF,
        wait_between_moves=1.0,
    )

    goto_table_center = goto.FromConstant(
        name="Goto table center",
        pose=create_stamped_pose(TABLE_CENTER_FRAME_CLUSTERED),
        ignore_depth=True,
    )

    ################## BOTTLE PART #################
    seq_collection_bottle_0 = create_collection_root(
        trash_name="Bottle 0",
        trash_frame=BOTTLE_FRAME,
        bucket_frame=PINK_BUCKET_FRAME,
    )
    seq_collection_bottle_1 = create_collection_root(
        trash_name="Bottle 1",
        trash_frame=BOTTLE_FRAME,
        bucket_frame=PINK_BUCKET_FRAME,
    )
    seq_collection_ladle_0 = create_collection_root(
        trash_name="Ladle 0",
        trash_frame=LADLE_FRAME,
        bucket_frame=YELLOW_BUCKET_FRAME,
    )
    seq_collection_ladle_1 = create_collection_root(
        trash_name="Ladle 1",
        trash_frame=LADLE_FRAME,
        bucket_frame=YELLOW_BUCKET_FRAME,
    )

    sel_bottle_0 = create_checked_collection_root(
        seq_collection_root=seq_collection_bottle_0
    )
    sel_bottle_1 = create_checked_collection_root(
        seq_collection_root=seq_collection_bottle_1
    )
    sel_ladle_0 = create_checked_collection_root(
        seq_collection_root=seq_collection_ladle_0
    )
    sel_ladle_1 = create_checked_collection_root(
        seq_collection_root=seq_collection_ladle_1
    )

    ############### ROTATION PARTS ###############

    seq_spin = create_spin_root()

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
            srv_start_vision,
            check_start_vision_succeeded,
            # seq_search,
            # goto_table_center,
            # par_search,
            # look_at_target,
            sel_bottle_0,
            sel_bottle_1,
            sel_ladle_0,
            sel_ladle_1,
            seq_spin,
            srv_end_vision,
            check_end_vision_succeeded,
        ]
    )

    return root
