import operator

import py_trees
import py_trees_ros
from bb_behavior_msgs.action import AlignAndCollect
from bb_perception_msgs.srv import ClusterTfSrv, GetObjectCount
from lifecycle_msgs.srv import ChangeState

from mission_planner_2.commons.detection_utils import (
    create_end_vision_req,
    create_start_vision_req,
)
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import (
    create_clustering_request,
    create_stamped_pose,
)
from mission_planner_2.commons.search import create_search_bot_layered_square_root
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.octagon.symbols import create_look_at_target_root
from mission_planner_2.trees.auv.octagon.trash import (
    create_align_actuate_surface_root,
    create_check_collections_changed_root,
    create_checked_collection_root,
    create_goto_table_centre_root,
    create_open_and_ascend_root,
    create_spin_root,
    create_trash_count_collection_root,
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

TRASH_COUNT_SERVICE = "/auv4/trash/object_count/toggle"
COLLECT_DURATION = 3.0

TOTAL_DURATION = 480
ALIGN_AND_COLLECT_TIMEOUT = 90.0
CONTROLLED_ASCENT_TIMEOUT = 30.0

CLUSTER_DURATION = 5
NUM_ROTATIONS = 6
STABILIZE_DURATION = 5
WAIT_BETWEEN_ROTATIONS = 3
LOOK_AT_TARGET_PAUSE_DURATION = 4
NUM_SQUARES = 1
OFFSET_COEFF = 1.0
DROP_Z_DISTANCE = 0.30

PICKUP_DEPTH_RATE = 0.05
PICKUP_XY_THRESHOLD = 0.03
DROP_DEPTH_RATE = 0.05
CONTROLLED_ASCENT_DEPTH_RATE = 0.05

SURFACE_DEPTH_THRESHOLD = 0.7
CONTROLLED_ASCENT_DEPTH_TOLERANCE = 0.05

CONTROLLED_SPIN_TOPIC = "/auv4/controlled_spin"
SPIN_YAW = 270.0

MAX_TABLE_CLUSTER_FAILURE = 3

CONTROLS_SRV_TOPIC = "/auv4/controls/controller"
GRABBER_ACTION_TOPIC = "/auv4/actuation/grabber"
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_CHOICE_KEY = "/global/choice_is_fish"
_START_VISION_KEY = fk("bin_start_vision")
_STOP_VISION_KEY = fk("bin_stop_vision")
_COLLECTION_RESULTS_KEY = fk("collection_results")
_OBJECT_TURN_KEY = fk("object_turn")
_TABLE_CLUSTER_FAILURE_COUNT_KEY = fk("table_cluster_failure_count")
_SPIN_ACTION_GOAL_KEY = fk("spin_action_goal")


def create_collection_root(
    trash_name: str,
    trash_frame: str,
    bucket_frame: str,
    trash_count_service: str,
):
    """Picks up trash, surfaces, looks at the target, and drops it in the bucket."""
    seq_trash = py_trees.composites.Sequence(
        name=trash_name,
        memory=True,
    )

    seq_trash_pick_up = create_align_actuate_surface_root(
        trash_name=trash_name,
        object_frame=trash_frame,
        command=AlignAndCollect.Goal.CLOSE,
        depth_rate=PICKUP_DEPTH_RATE,
        cluster_duration=CLUSTER_DURATION,
        xy_distance_threshold=PICKUP_XY_THRESHOLD,
        surface_depth_threshold=SURFACE_DEPTH_THRESHOLD,
        align_collect_timeout_seconds=ALIGN_AND_COLLECT_TIMEOUT,
        controlled_ascent_timeout_seconds=CONTROLLED_ASCENT_TIMEOUT,
    )

    goto_table_centre_drop = create_goto_table_centre_root(
        trash=trash_name,
        table_centre_frame=TABLE_CENTER_FRAME,
        table_centre_frame_clustered=TABLE_CENTER_FRAME_CLUSTERED,
        table_cluster_failure_count_key=_TABLE_CLUSTER_FAILURE_COUNT_KEY,
        cluster_duration=CLUSTER_DURATION,
        is_grabber_open=False,
    )

    seq_check_changed = create_check_collections_changed_root(
        initial_collection_result_key=_COLLECTION_RESULTS_KEY,
        trash_count_service=trash_count_service,
        collect_duration=COLLECT_DURATION,
    )

    seq_trash_drop = create_align_actuate_surface_root(
        trash_name=trash_name,
        object_frame=bucket_frame,
        command=AlignAndCollect.Goal.OPEN,
        depth_rate=DROP_DEPTH_RATE,
        cluster_duration=CLUSTER_DURATION,
        z_distance=DROP_Z_DISTANCE,
        surface_depth_threshold=SURFACE_DEPTH_THRESHOLD,
        align_collect_timeout_seconds=ALIGN_AND_COLLECT_TIMEOUT,
        controlled_ascent_timeout_seconds=CONTROLLED_ASCENT_TIMEOUT,
    )
    seq_trash.add_children(
        children=[
            seq_trash_pick_up,
            goto_table_centre_drop,
            seq_check_changed,
            seq_trash_drop,
        ]
    )
    return seq_trash


def create_search_root():
    """Rotate 360 degrees and cluster the poses of the fish and shark tags. At the same time,
    cluster the pose of the table center."""
    in_children = [
        FISH_FRAME,
        SHARK_FRAME,
        TABLE_CENTER_FRAME,
    ]
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
            operator=lambda x, y: x.success == y,  # type: ignore
        ),
    )

    init_table_clustering_count = py_trees.behaviours.SetBlackboardVariable(
        name="Initialise table clustering failure count",
        variable_name=_TABLE_CLUSTER_FAILURE_COUNT_KEY,
        variable_value=0,
        overwrite=True,
    )

    init_collection_results = py_trees.behaviours.SetBlackboardVariable(
        name="Initialise collection results",
        variable_name=_COLLECTION_RESULTS_KEY,
        variable_value=GetObjectCount.Response(
            success=False,
            num_bottles_on_table=0,
            num_ladles_on_table=0,
            num_objects_in_bucket=0,
            num_objects_uncollected=4,
        ),
        overwrite=True,
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
    seq_collection_bottle = create_collection_root(
        trash_name="bottle",
        trash_frame=BOTTLE_FRAME,
        bucket_frame=PINK_BUCKET_FRAME,
        trash_count_service=TRASH_COUNT_SERVICE,
    )

    seq_collection_ladle = create_collection_root(
        trash_name="ladle",
        trash_frame=LADLE_FRAME,
        bucket_frame=YELLOW_BUCKET_FRAME,
        trash_count_service=TRASH_COUNT_SERVICE,
    )

    seq_alternate = py_trees.composites.Sequence(
        name="Seq alternate trash pickup",
        memory=True,
    )

    seq_bottle = create_item_pickup_root(
        trash="bottle",
        seq_collection=seq_collection_bottle,
        is_first=True,
    )

    seq_ladle = create_item_pickup_root(
        trash="ladle",
        seq_collection=seq_collection_ladle,
        is_first=False,
    )

    seq_alternate.add_children(
        [
            seq_bottle,
            seq_ladle,
        ]
    )

    sel_main = py_trees.composites.Selector(
        name="Sel if not 0",
        memory=True,
    )

    seq_check_if_0 = py_trees.composites.Sequence(
        name="Check 0 sequence",
        memory=True,
    )

    seq_trash_counts = create_trash_count_collection_root(
        collection_result_key=_COLLECTION_RESULTS_KEY,
        trash_count_service=TRASH_COUNT_SERVICE,
        table_centre_frame=TABLE_CENTER_FRAME,
        table_centre_frame_clustered=TABLE_CENTER_FRAME_CLUSTERED,
        table_cluster_failure_count_key=_TABLE_CLUSTER_FAILURE_COUNT_KEY,
        cluster_duration=CLUSTER_DURATION,
        collect_duration=COLLECT_DURATION,
        trash="bottle",
        is_grabber_open=True,
    )

    check_0_on_table = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check 0 on table",
        check=py_trees.common.ComparisonExpression(
            variable=_COLLECTION_RESULTS_KEY,
            value=0,
            operator=lambda x, y: x.num_bottles_on_table + x.num_ladles_on_table == y,  # type: ignore
        ),
    )

    seq_check_if_0.add_children(
        [
            seq_trash_counts,
            check_0_on_table,
        ]
    )

    check_table_clustering_count = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check table clustering failure count",
        check=py_trees.common.ComparisonExpression(
            variable=_TABLE_CLUSTER_FAILURE_COUNT_KEY,
            value=MAX_TABLE_CLUSTER_FAILURE,
            operator=operator.eq,
        ),
    )

    force_fail_seq_alt = py_trees.decorators.SuccessIsFailure(
        name="Force fail seq alternate",
        child=seq_alternate,
    )

    sel_main.add_children(
        [
            seq_check_if_0,
            check_table_clustering_count,
            force_fail_seq_alt,
        ]
    )

    retry_collection = py_trees.decorators.Retry(
        name="Retry collections",
        child=sel_main,
        num_failures=100,
    )

    timeout_collection = py_trees.decorators.Timeout(
        name="Timeout collection",
        child=retry_collection,
        duration=TOTAL_DURATION,
    )

    sel_timeout = py_trees.composites.Selector(
        name="Timeout selector",
        memory=True,
    )

    seq_open_and_ascend = create_open_and_ascend_root(
        grabber_action_topic=GRABBER_ACTION_TOPIC,
        controls_srv_topic=CONTROLS_SRV_TOPIC,
        controlled_ascent_timeout_seconds=CONTROLLED_ASCENT_TIMEOUT,
        surface_depth_threshold=SURFACE_DEPTH_THRESHOLD,
        controlled_ascent_depth_tolerance=CONTROLLED_ASCENT_DEPTH_TOLERANCE,
        controlled_ascent_depth_rate=CONTROLLED_ASCENT_DEPTH_RATE,
        open_grabber_first=False,
    )

    sel_timeout.add_children(
        [
            timeout_collection,
            seq_open_and_ascend,
        ]
    )

    ############### ROTATION PARTS ###############

    seq_spin = create_spin_root(
        collection_result_key=_COLLECTION_RESULTS_KEY,
        spin_action_goal_key=_SPIN_ACTION_GOAL_KEY,
        trash_count_service=TRASH_COUNT_SERVICE,
        table_centre_frame=TABLE_CENTER_FRAME,
        table_centre_frame_clustered=TABLE_CENTER_FRAME_CLUSTERED,
        table_cluster_failure_count_key=_TABLE_CLUSTER_FAILURE_COUNT_KEY,
        cluster_duration=CLUSTER_DURATION,
        collect_duration=COLLECT_DURATION,
        controlled_spin_topic=CONTROLLED_SPIN_TOPIC,
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
            operator=lambda x, y: x.success == y,  # type: ignore
        ),
    )

    root.add_children(
        children=[
            srv_start_vision,
            check_start_vision_succeeded,
            init_table_clustering_count,
            init_collection_results,
            # seq_search,
            # goto_table_center,
            # par_search,
            # look_at_target,
            sel_timeout,
            seq_spin,
            srv_end_vision,
            check_end_vision_succeeded,
        ]
    )

    return root


def create_item_pickup_root(
    trash: str,
    seq_collection: py_trees.composites.Sequence,
    is_first: bool,
):
    check_value = "bottle" if trash == "ladle" else "ladle"

    seq_item = py_trees.composites.Sequence(
        name=f"Seq pick {trash}",
        memory=True,
    )

    seq_collect_trash_counts = create_trash_count_collection_root(
        collection_result_key=_COLLECTION_RESULTS_KEY,
        trash_count_service=TRASH_COUNT_SERVICE,
        table_centre_frame=TABLE_CENTER_FRAME,
        table_centre_frame_clustered=TABLE_CENTER_FRAME_CLUSTERED,
        table_cluster_failure_count_key=_TABLE_CLUSTER_FAILURE_COUNT_KEY,
        cluster_duration=CLUSTER_DURATION,
        collect_duration=COLLECT_DURATION,
        trash=trash,
        is_grabber_open=True,
    )

    sel_item = py_trees.composites.Selector(
        name=f"Select {trash}",
        memory=True,
    )

    check_wrong_item = py_trees.behaviours.CheckBlackboardVariableValue(
        name=f"Check is not {trash}",
        check=py_trees.common.ComparisonExpression(
            variable=_OBJECT_TURN_KEY,
            value=check_value,
            operator=operator.eq,
        ),
    )

    sel_trash = create_checked_collection_root(
        seq_collection_root=seq_collection,
        collection_result_key=_COLLECTION_RESULTS_KEY,
        trash=trash,
        controlled_ascent_depth_rate=CONTROLLED_ASCENT_DEPTH_RATE,
        controlled_ascent_depth_tolerance=CONTROLLED_ASCENT_DEPTH_TOLERANCE,
        surface_depth_threshold=SURFACE_DEPTH_THRESHOLD,
        controlled_ascent_timeout_seconds=CONTROLLED_ASCENT_TIMEOUT,
    )

    sel_item.add_children(
        [
            check_wrong_item,
            sel_trash,
        ]
    )

    set_turn = py_trees.behaviours.SetBlackboardVariable(
        name="set turn",
        variable_name=_OBJECT_TURN_KEY,
        variable_value=check_value,
        overwrite=True,
    )

    if is_first:
        children = [sel_item, set_turn]
    else:
        children = [seq_collect_trash_counts, sel_item, set_turn]

    seq_item.add_children(children)

    return seq_item
