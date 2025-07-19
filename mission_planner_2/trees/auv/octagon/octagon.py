import py_trees
import py_trees_ros
from bb_behavior_msgs.action import AlignAndCollect
from bb_perception_msgs.action import ClusterTf
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
    create_stamped_pose,
)
from mission_planner_2.commons.tf_checker import create_tf_checker_from_constant_root
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.octagon.helpers import view_frame_func
from mission_planner_2.trees.auv.octagon.symbols import create_reset_after_rubbish_root
from mission_planner_2.trees.auv.octagon.trash import create_trash_root
from std_srvs.srv import Trigger

# Generate namespace automatically from file path DONT set manually
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
VISION_SERVER_TOPIC = "/auv4/trash/manage_nodes"
ACTUATION_TOPIC = "/auv4/actuation/grabber"

CAMERA_FRAME = "auv4/front_cam_optical"

BOTTLE_0_FRAME_FROM_TABLE = "bottle_0/from_table"
BOTTLE_0_FRAME_FROM_ODOM = "bottle_0/from_odom"
BOTTLE_0_FRAME_CLUSTERED = "bottle_0/clustered"

BOTTLE_1_FRAME_FROM_TABLE = "bottle_1/from_table"
BOTTLE_1_FRAME_FROM_ODOM = "bottle_1/from_odom"
BOTTLE_1_FRAME_CLUSTERED = "bottle_1/clustered"

LADLE_0_FRAME_FROM_TABLE = "ladle_0/from_table"
LADLE_0_FRAME_FROM_ODOM = "ladle_0/from_odom"
LADLE_0_FRAME_CLUSTERED = "ladle_0/clustered"

LADLE_1_FRAME_FROM_TABLE = "ladle_1/from_table"
LADLE_1_FRAME_FROM_ODOM = "ladle_1/from_odom"
LADLE_1_FRAME_CLUSTERED = "ladle_1/clustered"

PINK_BUCKET_FRAME_FROM_TABLE = "pink_bucket/from_table"
PINK_BUCKET_FRAME_FROM_ODOM = "pink_bucket/from_odom"
PINK_BUCKET_FRAME_CLUSTERED = "pink_bucket/clustered"

YELLOW_BUCKET_FRAME_FROM_TABLE = "yellow_bucket/from_table"
YELLOW_BUCKET_FRAME_FROM_ODOM = "yellow_bucket/from_odom"
YELLOW_BUCKET_FRAME_CLUSTERED = "yellow_bucket/clustered"

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
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_CHOICE_KEY = fk("choice")
_GO_SURFACE_FRAME_KEY = fk("go_surface_frame")
_START_VISION_KEY = fk("bin_start_vision")
_STOP_VISION_KEY = fk("bin_stop_vision")
_FISH_TF_KEY = fk("fish_tf")
_SHARK_TF_KEY = fk("shark_tf")


def create_collection_root(
    trash_name: str,
    trash_frame_depth_from_table: str,
    trash_frame_depth_from_odom: str,
    trash_frame_clustered: str,
    bucket_frame_depth_from_table: str,
    bucket_frame_depth_from_odom: str,
    bucket_frame_clustered: str,
):
    seq_trash = py_trees.composites.Sequence(
        name=trash_name,
        memory=True,
    )
    seq_trash_pick_up = create_trash_root(
        trash_frame_depth_from_table,
        trash_frame_depth_from_odom,
        trash_frame_clustered,
        trash_name,
        depth_threshold=0.1,
        cluster_duration=CLUSTER_DURATION,
        z_distance=0.20,
    )
    seq_reset_trash_pick_up = create_reset_after_rubbish_root(
        fish_frame=FISH_FRAME,
        fish_frame_clustered=FISH_FRAME_CLUSTERED,
        shark_frame=SHARK_FRAME,
        shark_frame_clustered=SHARK_FRAME_CLUSTERED,
        fish_view_frame=FISH_VIEW_FRAME,
        fish_view_frame_hardcoded=FISH_VIEW_FRAME_HARDCODED,
        shark_view_frame=SHARK_VIEW_FRAME,
        shark_view_frame_hardcoded=SHARK_VIEW_FRAME_HARDCODED,
        cluster_duration=CLUSTER_DURATION,
        choice_key=_CHOICE_KEY,
        rubbish_name=trash_name,
    )
    cluster_table_centre = py_trees_ros.action_clients.FromConstant(
        name="Cluster centre",
        action_type=ClusterTf,
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
    seq_trash_drop = create_trash_root(
        bucket_frame_depth_from_table,
        bucket_frame_depth_from_odom,
        bucket_frame_clustered,
        trash_name=trash_name,
        depth_threshold=0.1,
        cluster_duration=CLUSTER_DURATION,
        command=AlignAndCollect.Goal.OPEN,
        z_distance=0.30,
    )
    seq_trash.add_children(
        children=[
            seq_trash_pick_up,
            # seq_reset_trash_pick_up,
            cluster_table_centre,
            goto_table_centre,
            stabilize_before_drop,
            seq_trash_drop,
        ]
    )
    return seq_trash


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

    par_search_tag = py_trees.composites.Parallel(
        name="Search tag",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )

    goto_n_search_poses = goto.NFromConstant(
        name="Goto search poses",
        poses=[
            create_stamped_pose(
                frame_id="auv4/base_link_ned",
                yaw=360.0 / NUM_ROTATIONS,
            )
            for _ in range(NUM_ROTATIONS)
        ],
        anchor_frame_name="auv4/base_link_ned",
        specified_heading=True,
        wait_between_moves_sec=10.0,
    )

    cluster_tags = py_trees_ros.actions.ActionClient(
        name="Cluster tags",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=[FISH_FRAME, SHARK_FRAME],
            out_children=[FISH_FRAME_CLUSTERED, SHARK_FRAME_CLUSTERED],
            duration=CLUSTER_DURATION,
            use_cache=False,
            persistent=True,
        ),
    )

    par_search_tag.add_children(
        [
            goto_n_search_poses,
            cluster_tags,
        ]
    )

    symbol_tf_checker = create_tf_checker_from_constant_root(
        start_frames=[FISH_FRAME_CLUSTERED, SHARK_FRAME_CLUSTERED],
        update_keys=[_FISH_TF_KEY, _SHARK_TF_KEY],
        end_frames=["world_ned", "world_ned"],
        fallback_val=[FISH_VIEW_FRAME_HARDCODED, SHARK_VIEW_FRAME_HARDCODED],
    )

    dynamic_set_surface_pose_frame = DynamicSetBlackboard(
        name="select surface frame",
        key=[_CHOICE_KEY, _FISH_TF_KEY, _SHARK_TF_KEY],
        update_key=_GO_SURFACE_FRAME_KEY,
        overwrite=True,
        func=lambda choice, fish_tf, shark_tf: view_frame_func(
            choice, fish_tf, shark_tf, FISH_VIEW_FRAME, SHARK_VIEW_FRAME
        ),
    )

    ################## BOTTLE PART #################
    seq_bottle_0 = create_collection_root(
        trash_name="Bottle 0",
        trash_frame_depth_from_table=BOTTLE_0_FRAME_FROM_TABLE,
        trash_frame_depth_from_odom=BOTTLE_0_FRAME_FROM_ODOM,
        trash_frame_clustered=BOTTLE_0_FRAME_CLUSTERED,
        bucket_frame_depth_from_table=PINK_BUCKET_FRAME_FROM_TABLE,
        bucket_frame_depth_from_odom=PINK_BUCKET_FRAME_FROM_ODOM,
        bucket_frame_clustered=PINK_BUCKET_FRAME_CLUSTERED,
    )
    seq_ladle_0 = create_collection_root(
        trash_name="Ladle 0",
        trash_frame_depth_from_table=LADLE_0_FRAME_FROM_TABLE,
        trash_frame_depth_from_odom=LADLE_0_FRAME_FROM_ODOM,
        trash_frame_clustered=LADLE_0_FRAME_CLUSTERED,
        bucket_frame_depth_from_table=YELLOW_BUCKET_FRAME_FROM_TABLE,
        bucket_frame_depth_from_odom=YELLOW_BUCKET_FRAME_FROM_ODOM,
        bucket_frame_clustered=YELLOW_BUCKET_FRAME_CLUSTERED,
    )

    ############### ROTATION PARTS ###############
    # TODO: check the tfs with the within_dist to the basket to count before rotating
    goto_rotations = goto.NFromConstant(
        name="Go to rotations",
        poses=[
            create_stamped_pose(frame_id="auv4/base_link_ned", yaw=360.0),
            create_stamped_pose(frame_id="auv4/base_link_ned", yaw=360.0),
            create_stamped_pose(frame_id="auv4/base_link_ned", yaw=360.0),
            create_stamped_pose(frame_id="auv4/base_link_ned", yaw=360.0),
        ],
        wait_between_moves_sec=10.0,
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
            # par_search_tag,
            # symbol_tf_checker,
            # dynamic_set_surface_pose_frame,
            seq_bottle_0,
            seq_ladle_0,
            # goto_rotations,
            # srv_end_vision,
            # check_end_vision_succeeded,
        ]
    )

    return root
