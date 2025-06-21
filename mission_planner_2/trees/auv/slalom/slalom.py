import operator

import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from rclpy.qos import qos_profile_system_default
from std_srvs.srv import Trigger

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
)
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.slalom.channel_movement import (
    create_channel_movement_one_root,
    create_channel_movement_two_root,
    create_channel_movement_zero_root,
)

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

########################## UPDATE CONSTANTS HERE #########################
BASE_LINK_FRAME = "auv4/base_link_ned"
WORLD_FRAME = "world_ned"
CHANNEL_PAIR_ZERO_FRAME = "slalom_layer_0"
CHANNEL_PAIR_ONE_FRAME = "slalom_layer_1"
CHANNEL_PAIR_TWO_FRAME = "slalom_layer_2"

CHANNEL_PAIR_ZERO_FRAME_CLUSTERED = CHANNEL_PAIR_ZERO_FRAME + "/clustered"
CHANNEL_PAIR_ONE_FRAME_CLUSTERED = CHANNEL_PAIR_ONE_FRAME + "/clustered"
CHANNEL_PAIR_TWO_FRAME_CLUSTERED = CHANNEL_PAIR_TWO_FRAME + "/clustered"

SLALOM_ONE_FROM_ZERO_HARDCODED = "slalom_layer_1/hardcoded"
SLALOM_TWO_FROM_ONE_HARDCODED_HARDCODED = "slalom_layer_2/hardcoded/hardcoded"

TRANSFORM_TIMEOUT_DURATION = 5.0
CLUSTER_VIEW_DURATION = 40

"""
For sim.
FIRST_VIEW = {"position_x": 6.0, "position_y": -0.8, "position_z": 1.0, "yaw": -90.0}
SECOND_VIEW = {"position_x": 5.0, "position_y": -0.8, "position_z": 1.0, "yaw": -90.0}
THIRD_VIEW = {"position_x": 7.0, "position_y": -0.8, "position_z": 1.0, "yaw": -90.0}
"""

FIRST_VIEW = {"position_x": 0.0, "position_y": 0.0, "position_z": 0.0, "yaw": 0.0}
SECOND_VIEW = {"position_x": 0.0, "position_y": -1.2, "position_z": 0.0, "yaw": 0.0}
THIRD_VIEW = {"position_x": 0.0, "position_y": 2.4, "position_z": 0.0, "yaw": 0.0}

# set by  gate task if there change must change here too
IS_LEFT_KEY = "/global/is_left_side"  # Global key for left option or not
#########################################################################

_CHANNEL_ZERO_KEY = fk("channel_pair_zero_tf")
_CHANNEL_ONE_KEY = fk("channel_pair_one_tf")
_CHANNEL_TWO_KEY = fk("channel_pair_two_tf")
_CREATE_POSE_FUNC_KEY = fk("create_pose_func")  # key for pose creation function
_MISSING_TRANSFORMS_KEY = fk("missing_transforms")  # key for missing transforms


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
    )

    move_and_cluster_par = py_trees.composites.Parallel(
        name="Move to different views and cluster",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )

    cluster_action = py_trees_ros.action_clients.FromConstant(
        name="Cluster slalom transforms",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf_multi",
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
            duration=CLUSTER_VIEW_DURATION,
        ),
    )

    seq_move_and_cluster = py_trees.composites.Sequence(
        name="Move to different views and cluster",
        memory=True,
        children=[
            move_view_one,
            py_trees.timers.Timer(duration=10),
            move_view_two,
            py_trees.timers.Timer(duration=10),
            move_view_three,
            py_trees.timers.Timer(duration=10),
        ],
    )

    move_and_cluster_par.add_children([cluster_action, seq_move_and_cluster])

    # Initialize the number of missing transforms in blackboard to be used by DynamicSetBlackboard
    init_missing_transforms = py_trees.behaviours.SetBlackboardVariable(
        name="Initialize missing transforms",
        variable_name=fk("missing_transforms"),
        variable_value=0,
        overwrite=True,
    )

    # There is now A need to read/write the transforms from the blackboard (changed)
    check_transform_zero = py_trees_ros.transforms.ToBlackboard(
        name="Write channel pair zero transform",
        variable_name=_CHANNEL_ZERO_KEY,
        target_frame=CHANNEL_PAIR_ZERO_FRAME_CLUSTERED,
        source_frame=BASE_LINK_FRAME,
        qos_profile=qos_profile_system_default,
    )

    check_transform_one = py_trees_ros.transforms.ToBlackboard(
        name="Write channel pair one transform",
        variable_name=_CHANNEL_ONE_KEY,
        target_frame=CHANNEL_PAIR_ONE_FRAME_CLUSTERED,
        source_frame=BASE_LINK_FRAME,
        qos_profile=qos_profile_system_default,
    )

    check_transform_two = py_trees_ros.transforms.ToBlackboard(
        name="Write channel pair two transform",
        variable_name=_CHANNEL_TWO_KEY,
        target_frame=CHANNEL_PAIR_TWO_FRAME_CLUSTERED,
        source_frame=BASE_LINK_FRAME,
        qos_profile=qos_profile_system_default,
    )

    update_missing_transforms_one = DynamicSetBlackboard(
        name="Update missing transforms one",
        key=_MISSING_TRANSFORMS_KEY,
        update_key=_MISSING_TRANSFORMS_KEY,
        overwrite=True,
        func=lambda x: x + 1,
    )

    update_missing_transforms_two = DynamicSetBlackboard(
        name="Update missing transforms two",
        key=_MISSING_TRANSFORMS_KEY,
        update_key=_MISSING_TRANSFORMS_KEY,
        overwrite=True,
        func=lambda x: x + 1,
    )

    update_missing_transforms_three = DynamicSetBlackboard(
        name="Update missing transforms three",
        key=_MISSING_TRANSFORMS_KEY,
        update_key=_MISSING_TRANSFORMS_KEY,
        overwrite=True,
        func=lambda x: x + 1,
    )

    # Structure the check as a fallback that will update the number of missing transforms
    check_and_update_fallback_one = py_trees.composites.Selector(
        name="Check and update missing transforms one",
        memory=True,
        children=[
            py_trees.decorators.Timeout(
                name="Timeout for channel pair one",
                child=check_transform_zero,
                duration=TRANSFORM_TIMEOUT_DURATION,
            ),
            update_missing_transforms_one,
        ],
    )

    check_and_update_fallback_two = py_trees.composites.Selector(
        name="Check and update missing transforms two",
        memory=True,
        children=[
            py_trees.decorators.Timeout(
                name="Timeout for channel pair two",
                child=check_transform_one,
                duration=TRANSFORM_TIMEOUT_DURATION,
            ),
            update_missing_transforms_two,
        ],
    )

    check_and_update_fallback_three = py_trees.composites.Selector(
        name="Check and update missing transforms three",
        memory=True,
        children=[
            py_trees.decorators.Timeout(
                name="Timeout for channel pair three",
                child=check_transform_two,
                duration=TRANSFORM_TIMEOUT_DURATION,
            ),
            update_missing_transforms_three,
        ],
    )

    # Sequence to check transforms and update the number of missing transforms
    seq_check_transforms = py_trees.composites.Sequence(
        name="Check transforms",
        memory=True,
        children=[
            init_missing_transforms,
            check_and_update_fallback_one,
            check_and_update_fallback_two,
            check_and_update_fallback_three,
        ],
    )

    # Generate movement options based on the number of missing transforms, generation done in compile time, execution done in runtime
    move_channel_one = create_channel_movement_zero_root(
        slalom_frame_zero_clustered=CHANNEL_PAIR_ZERO_FRAME_CLUSTERED,
        slalom_frame_one_clustered=CHANNEL_PAIR_ONE_FRAME_CLUSTERED,
        slalom_frame_two_clustered=CHANNEL_PAIR_TWO_FRAME_CLUSTERED,
        create_func_key=_CREATE_POSE_FUNC_KEY,
    )
    move_channel_two = create_channel_movement_one_root(
        slalom_frame_zero_clustered=CHANNEL_PAIR_ZERO_FRAME_CLUSTERED,
        slalom_frame_one_clustered=CHANNEL_PAIR_ONE_FRAME_CLUSTERED,
        slalom_frame_zero_key=_CHANNEL_ZERO_KEY,
        slalom_frame_one_key=_CHANNEL_ONE_KEY,
        slalom_one_from_zero_hardcoded=SLALOM_ONE_FROM_ZERO_HARDCODED,
        slalom_two_from_one_hardcoded=SLALOM_TWO_FROM_ONE_HARDCODED_HARDCODED,
        create_func_key=_CREATE_POSE_FUNC_KEY,
    )
    move_channel_three = create_channel_movement_two_root(
        slalom_frame_zero_clustered=CHANNEL_PAIR_ZERO_FRAME_CLUSTERED,
        slalom_one_from_zero_hardcoded=SLALOM_ONE_FROM_ZERO_HARDCODED,
        slalom_two_from_one_hardcoded=SLALOM_TWO_FROM_ONE_HARDCODED_HARDCODED,
        create_func_key=_CREATE_POSE_FUNC_KEY,
    )

    # helper function to check num missing tfs
    check = lambda num_missing: py_trees.common.ComparisonExpression(
        variable=_MISSING_TRANSFORMS_KEY,
        value=num_missing,
        operator=operator.eq,
    )

    check_missing_transforms_one = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check missing zero transforms",
        check=check(0),
    )
    check_missing_transforms_two = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check missing one transform", check=check(1)
    )
    check_missing_transforms_three = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check missing two transforms", check=check(2)
    )

    # Selector to choose the movement strategy based on the number of missing transforms
    select_movement_strategy = py_trees.composites.Selector(
        name="Select movement strategy",
        memory=True,
        children=[
            py_trees.composites.Sequence(
                name="Zero missing transforms",
                memory=True,
                children=[check_missing_transforms_one, move_channel_one],
            ),
            py_trees.composites.Sequence(
                name="One missing transform",
                memory=True,
                children=[check_missing_transforms_two, move_channel_two],
            ),
            py_trees.composites.Sequence(
                name="Two missing transforms",
                memory=True,
                children=[check_missing_transforms_three, move_channel_three],
            ),
        ],
    )

    sel_left_right = py_trees.composites.Selector(
        name="Left or right side",
        memory=True,
    )

    seq_set_left = py_trees.composites.Sequence(
        name="Set left side",
        memory=True,
    )

    check_is_left = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check is left side",
        check=py_trees.common.ComparisonExpression(
            variable=IS_LEFT_KEY,
            value=True,
            operator=operator.eq,
        ),
    )

    seq_set_left.add_children(
        [
            check_is_left,
            py_trees.behaviours.SetBlackboardVariable(
                name="Seq set create func left side",
                variable_name=_CREATE_POSE_FUNC_KEY,
                variable_value=_create_slalom_left_pose,
                overwrite=True,
            ),
        ]
    )

    sel_left_right.add_children(
        [
            seq_set_left,
            py_trees.behaviours.SetBlackboardVariable(
                name="Set set create func right side",
                variable_name=_CREATE_POSE_FUNC_KEY,
                variable_value=_create_slalom_right_pose,
                overwrite=True,
            ),
        ]
    )

    goto_pass_through = goto.FromConstant(
        "Pass through gate",
        pose=create_stamped_pose("auv4/base_link_ned", position_x=2.0),
    )

    root.add_children(
        [
            sel_left_right,
            move_and_cluster_par,
            seq_check_transforms,
            select_movement_strategy,
            py_trees.timers.Timer(name="timer", duration=10.0),
            goto_pass_through,
        ]
    )

    return root
