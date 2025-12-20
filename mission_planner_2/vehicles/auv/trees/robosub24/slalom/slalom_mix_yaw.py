import operator

import py_trees
import py_trees_ros
from lifecycle_msgs.srv import ChangeState
from std_srvs.srv import SetBool

from mission_planner_2.common.core import shared_action_client
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
from mission_planner_2.vehicles.auv.trees.robosub24.goto import goto
from mission_planner_2.vehicles.auv.trees.robosub24.slalom.channel_movement_mix import (
    create_movement_strategy_root,
)

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

RECLUSTER = False

if RECLUSTER:
    pass
else:
    pass


########################## UPDATE CONSTANTS HERE #########################
VISION_SERVER_TOPIC = "/auv4/slalom/manage_nodes"
DEPTH_ANYTHING_SERVER_TOPIC = "/auv4/slalom/manage_components"

BASE_LINK_FRAME = "auv4/base_link_ned"
WORLD_FRAME = "world_ned"
CHANNEL_PAIR_ZERO_NEAR_FRAME = "slalom_layer_near_0"
CHANNEL_PAIR_ZERO_FRAME = "slalom_layer_0"
CHANNEL_PAIR_ONE_FRAME = "slalom_layer_1"
CHANNEL_PAIR_TWO_FRAME = "slalom_layer_2"

CHANNEL_PAIR_ZERO_FRAME_CLUSTERED = CHANNEL_PAIR_ZERO_FRAME + "/clustered"
CHANNEL_PAIR_ONE_FRAME_CLUSTERED = CHANNEL_PAIR_ONE_FRAME + "/clustered"
CHANNEL_PAIR_TWO_FRAME_CLUSTERED = CHANNEL_PAIR_TWO_FRAME + "/clustered"

CHANNEL_CENTRE_FRAME = "slalom/centre"

SLALOM_ONE_FROM_ZERO_HARDCODED = "slalom_layer_1/hardcoded"
SLALOM_TWO_FROM_ONE_HARDCODED_HARDCODED = "slalom_layer_2/hardcoded/hardcoded"

TRANSFORM_TIMEOUT_DURATION = 5.0
CLUSTER_VIEW_DURATION = 5
WAIT_BETWEEN_MOVES_SEC = 0.1
MIN_CLUSTER_SIZE = 10

DEPTH_OVERRIDE_VALUE = 0.9
CLUSTERING_SERVICE_NAME = "/auv4/cluster_tfs_multi_srv"
LAYER_ZERO_NEAR = "slalom_layer_near_0"
LAYER_ONE_NEAR = "slalom_layer_near_1"
LAYER_TWO_NEAR = "slalom_layer_near_2"
CLUSTERING_IN_CHILDREN_NEAR = [
    LAYER_ZERO_NEAR,
    LAYER_ONE_NEAR,
    LAYER_TWO_NEAR,
]
NUM_SWEEP_STOPS = 3
SWEEP_ANGLE_DEGREES = 15.0

"""
For sim.
FIRST_VIEW = {"position_x": 6.0, "position_y": -0.8, "position_z": 1.0, "yaw": -90.0}
SECOND_VIEW = {"position_x": 5.0, "position_y": -0.8, "position_z": 1.0, "yaw": -90.0}
THIRD_VIEW = {"position_x": 7.0, "position_y": -0.8, "position_z": 1.0, "yaw": -90.0}
"""

FIRST_VIEW = {"position_x": 0.0, "position_y": 0.0, "position_z": 0.0, "yaw": 0.0}
SECOND_VIEW = {"position_x": 0.0, "position_y": -0.6, "position_z": 0.0, "yaw": 0.0}
THIRD_VIEW = {"position_x": 0.0, "position_y": 1.2, "position_z": 0.0, "yaw": 0.0}

# set by  gate task if there change must change here too
IS_LEFT_KEY = "/global/is_left_side"  # Global key for left option or not
#########################################################################

_CHANNEL_ZERO_KEY = fk("channel_pair_zero_tf")
_CHANNEL_ONE_KEY = fk("channel_pair_one_tf")
_CHANNEL_TWO_KEY = fk("channel_pair_two_tf")
# key for pose creation function
_CREATE_POSE_FUNC_KEY = fk("create_pose_func")
# key for missing transforms
_MISSING_TRANSFORMS_KEY = fk("missing_transforms")
_START_VISION_KEY = fk("slalom_start_vision")
_STOP_VISION_KEY = fk("slalom_stop_vision")
_START_COMPONENTS_KEY = fk("slalom_start_components")
_STOP_COMPONENTS_KEY = fk("slalom_stop_components")


def create_move_slalom_centre_root():
    root = py_trees.composites.Selector(name="Move to centre post", memory=True)

    seq_move_centre_left = py_trees.composites.Sequence(
        name="Move post left",
        memory=True,
    )

    check_is_left = py_trees.behaviours.CheckBlackboardVariableValue(
        name="check is left",
        check=py_trees.common.ComparisonExpression(
            variable=IS_LEFT_KEY,
            value=True,
            operator=operator.eq,
        ),
    )

    goto_left = goto.FromConstant(
        name="Move to post left",
        pose=create_stamped_pose(
            frame_id=BASE_LINK_FRAME,
            position_x=2.0,
            position_y=0.75,
        ),
        depth_override_value=DEPTH_OVERRIDE_VALUE,
    )

    seq_move_centre_left.add_children(
        [
            check_is_left,
            goto_left,
        ]
    )

    goto_right = goto.FromConstant(
        name="Move to post right",
        pose=create_stamped_pose(
            frame_id=BASE_LINK_FRAME,
            position_x=2.0,
            position_y=-0.75,
        ),
        depth_override_value=DEPTH_OVERRIDE_VALUE,
    )

    root.add_children(
        [
            seq_move_centre_left,
            goto_right,
        ]
    )

    return root


def _create_goto_sweep_and_cluster(
    stop: int,
    angle: float,
):
    root = py_trees.composites.Sequence(
        name="Goto and cluster",
        memory=True,
    )

    goto_sweep = goto.FromConstant(
        name=f"{stop} sweep at angle {angle}",
        pose=create_stamped_pose(
            frame_id=BASE_LINK_FRAME,
            yaw=angle,
            use_radians=False,
        ),
        depth_override_value=DEPTH_OVERRIDE_VALUE,
    )

    recluster_action = shared_action_client.FromConstant(
        name=f"Recluster for stop {stop} at angle {angle}",
        shared_action=AUVSharedAction.CLUSTER_MULTI,
        action_goal=create_clustering_goal(
            # in_children=[
            #     CHANNEL_PAIR_ZERO_FRAME,
            #     CHANNEL_PAIR_ONE_FRAME,
            #     CHANNEL_PAIR_TWO_FRAME,
            # ],
            in_children=CLUSTERING_IN_CHILDREN_NEAR,
            out_children=[
                CHANNEL_PAIR_ZERO_FRAME_CLUSTERED,
                CHANNEL_PAIR_ONE_FRAME_CLUSTERED,
                CHANNEL_PAIR_TWO_FRAME_CLUSTERED,
            ],
            duration=CLUSTER_VIEW_DURATION,
            persistent=True,
        ),
    )

    root.add_children(
        [
            goto_sweep,
            recluster_action,
        ]
    )

    return root


def create_sweeps_and_recluster_root(
    num_stops: int,
    sweep_angle: float,
):
    root = py_trees.composites.Sequence(
        name=f"Sweep and recluster stops",
        memory=True,
    )
    sweep_and_cluster_sequences = [
        _create_goto_sweep_and_cluster(
            stop=stop,
            angle=(-sweep_angle + stop * (2 * sweep_angle) / (num_stops - 1)),
        )
        for stop in range(num_stops)
    ]

    root.add_children(sweep_and_cluster_sequences)

    return root


def create_slalom_mix_yaw_root():
    """
    Create the root of the slalom tree.
    """

    """
    In this tree, the AUV will:
    1. Move to various views (3) and at each view, invoke the clustering action call.
    2. After completing the 3 views, the AUV will check if the clustered transforms (quantity) are available. The ideal is that all 3 transforms are available.
    3. We create fallback trees based on the number of missing transforms in advance (0, 1 or 2). Based on the number of missing transforms, the AUV will execute the appropriate movement strategy.
    3a. Within each fallback tree, we would have also defined some hardcoded transforms that the AUV should move to, in the event of missing transforms.
        For example, 1 missing transform would mean that the AUV will move to the first 2 transforms that are available, and then move to the hardcoded transform from transform 2.
        2 missing transforms would mean that the AUV will move to the first transform that is available, and then move to the hardcoded transform from transform 1 and to the hardcoded transform from hardcoded transform 2.
    """

    root = py_trees.composites.Sequence(
        name="Slalom Task",
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
            operator=lambda x, y: operator.eq(x.success, y),
        ),
    )

    srv_load_depth_anything = py_trees_ros.service_clients.FromConstant(
        name="Start depth anything",
        service_name=DEPTH_ANYTHING_SERVER_TOPIC,
        service_type=SetBool,
        service_request=SetBool.Request(data=True),
        key_response=_START_COMPONENTS_KEY,
    )
    check_start_depth_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify start depth anything succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=_START_COMPONENTS_KEY,
            value=True,
            operator=lambda x, y: operator.eq(x.success, y),
        ),
    )

    seq_sweep_and_cluster = create_sweeps_and_recluster_root(
        num_stops=NUM_SWEEP_STOPS,
        sweep_angle=SWEEP_ANGLE_DEGREES,
    )

    seq_movement_strategy = create_movement_strategy_root()

    sel_goto_post = create_move_slalom_centre_root()

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
            operator=lambda x, y: operator.eq(x.success, y),
        ),
    )

    srv_unload_depth_anything = py_trees_ros.service_clients.FromConstant(
        name="Stop depth anything",
        service_name=DEPTH_ANYTHING_SERVER_TOPIC,
        service_type=SetBool,
        service_request=SetBool.Request(data=False),
        key_response=_STOP_COMPONENTS_KEY,
    )
    check_end_depth_anything_succeeded = (
        py_trees.behaviours.CheckBlackboardVariableValue(
            name="Verify end depth anything succeeded",
            check=py_trees.common.ComparisonExpression(
                variable=_STOP_COMPONENTS_KEY,
                value=True,
                operator=lambda x, y: operator.eq(x.success, y),
            ),
        )
    )

    root.add_children(
        [
            srv_start_vision,
            check_start_vision_succeeded,
            srv_load_depth_anything,
            check_start_depth_succeeded,
            seq_sweep_and_cluster,
            seq_movement_strategy,
            sel_goto_post,
            srv_end_vision,
            check_end_vision_succeeded,
            srv_unload_depth_anything,
            check_end_depth_anything_succeeded,
        ]
    )

    return root
