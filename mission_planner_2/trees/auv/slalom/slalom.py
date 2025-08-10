import operator

import py_trees
import py_trees_ros
from lifecycle_msgs.srv import ChangeState
from std_srvs.srv import SetBool

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
    create_stamped_pose,
)
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.slalom.channel_movement_mix import (
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

MOVE_VIEW_DEPTH = 0.3
CLUSTERING_SERVICE_NAME = "/auv4/cluster_tfs_multi_srv"

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


def _create_slalom_left_pose(frame_id: str):
    """
    Create a PoseStamped for the left side of the slalom.
    """
    return create_stamped_pose(
        frame_id=frame_id,
        position_x=0.75,
        position_y=0.3,
        position_z=0.0,
        roll=-90.0,
        pitch=-90.0,
        yaw=0.0,  # Facing left
    )


def _create_slalom_right_pose(frame_id: str):
    """
    Create a PoseStamped for the right side of the slalom.
    """
    return create_stamped_pose(
        frame_id=frame_id,
        position_x=2.25,
        position_y=0.3,
        position_z=0.0,
        roll=-90.0,
        pitch=-90.0,
        yaw=0.0,
    )


def create_slalom_root():
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

    cluster_action_1 = shared_action_client.FromConstant(
        name="Initial cluster 1",
        shared_action=SharedAction.CLUSTER_MULTI,
        action_goal=create_clustering_goal(
            in_children=[
                CHANNEL_PAIR_ZERO_FRAME,
                CHANNEL_PAIR_ONE_FRAME,
                CHANNEL_PAIR_TWO_FRAME,
            ],
            out_children=[
                CHANNEL_PAIR_ZERO_FRAME_CLUSTERED,
                CHANNEL_PAIR_ONE_FRAME_CLUSTERED,
                CHANNEL_PAIR_TWO_FRAME_CLUSTERED,
            ],
            min_cluster_size=MIN_CLUSTER_SIZE,
            min_samples=MIN_CLUSTER_SIZE,
            persistent=True,
            duration=CLUSTER_VIEW_DURATION,
        ),
    )
    cluster_action_2 = shared_action_client.FromConstant(
        name="Initial cluster 2",
        shared_action=SharedAction.CLUSTER_MULTI,
        action_goal=create_clustering_goal(
            in_children=[
                CHANNEL_PAIR_ZERO_FRAME,
                CHANNEL_PAIR_ONE_FRAME,
                CHANNEL_PAIR_TWO_FRAME,
            ],
            out_children=[
                CHANNEL_PAIR_ZERO_FRAME_CLUSTERED,
                CHANNEL_PAIR_ONE_FRAME_CLUSTERED,
                CHANNEL_PAIR_TWO_FRAME_CLUSTERED,
            ],
            min_cluster_size=MIN_CLUSTER_SIZE,
            min_samples=MIN_CLUSTER_SIZE,
            persistent=True,
            duration=CLUSTER_VIEW_DURATION,
        ),
    )
    cluster_action_3 = shared_action_client.FromConstant(
        name="Initial cluster 3",
        shared_action=SharedAction.CLUSTER_MULTI,
        action_goal=create_clustering_goal(
            in_children=[
                CHANNEL_PAIR_ZERO_FRAME,
                CHANNEL_PAIR_ONE_FRAME,
                CHANNEL_PAIR_TWO_FRAME,
            ],
            out_children=[
                CHANNEL_PAIR_ZERO_FRAME_CLUSTERED,
                CHANNEL_PAIR_ONE_FRAME_CLUSTERED,
                CHANNEL_PAIR_TWO_FRAME_CLUSTERED,
            ],
            min_cluster_size=MIN_CLUSTER_SIZE,
            min_samples=MIN_CLUSTER_SIZE,
            persistent=True,
            duration=CLUSTER_VIEW_DURATION,
        ),
    )

    # TODO: Update the frame and pose to move to (consider doing it similar to octagon task search seq)
    move_view_one = goto.FromConstant(
        name="move one",
        pose=create_stamped_pose(
            BASE_LINK_FRAME,
            position_x=FIRST_VIEW["position_x"],
            position_y=FIRST_VIEW["position_y"],
            position_z=FIRST_VIEW["position_z"],
            yaw=FIRST_VIEW["yaw"],
        ),
        depth_override_value=MOVE_VIEW_DEPTH,
    )

    move_view_two = goto.FromConstant(
        name="move two",
        pose=create_stamped_pose(
            BASE_LINK_FRAME,
            position_x=SECOND_VIEW["position_x"],
            position_y=SECOND_VIEW["position_y"],
            position_z=SECOND_VIEW["position_z"],
            yaw=SECOND_VIEW["yaw"],
        ),
        depth_override_value=MOVE_VIEW_DEPTH,
    )

    move_view_three = goto.FromConstant(
        name="move three",
        pose=create_stamped_pose(
            BASE_LINK_FRAME,
            position_x=THIRD_VIEW["position_x"],
            position_y=THIRD_VIEW["position_y"],
            position_z=THIRD_VIEW["position_z"],
            yaw=THIRD_VIEW["yaw"],
        ),
        depth_override_value=MOVE_VIEW_DEPTH,
    )

    seq_move_and_cluster = py_trees.composites.Sequence(
        name="Move to different views and cluster",
        memory=True,
        children=[
            move_view_one,
            cluster_action_1,
            move_view_two,
            cluster_action_2,
            move_view_three,
            cluster_action_3,
        ],
    )

    seq_movement_strategy = create_movement_strategy_root()

    goto_pass_through = goto.FromConstant(
        "Pass through gate",
        pose=create_stamped_pose("auv4/base_link_ned", position_x=2.0),
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
            seq_move_and_cluster,
            seq_movement_strategy,
            # goto_pass_through,
            srv_end_vision,
            check_end_vision_succeeded,
            srv_unload_depth_anything,
            check_end_depth_anything_succeeded,
        ]
    )

    return root
