import operator

import py_trees
import py_trees_ros
from rclpy.qos import qos_profile_system_default

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.slalom.channel_movement import (
    create_channel_movement_root,
)
from mission_planner_2.trees.auv.slalom.move_and_cluster import (
    create_move_and_cluster_root,
)
from mission_planner_2.trees.auv.slalom.move_to_task import create_move_to_task_root

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

########################## UPDATE CONSTANTS HERE #########################
BASE_LINK_FRAME = "auv4/base_link_ned"
CHANNEL_PAIR_ONE_FRAME = "slalom_layer_0"
CHANNEL_PAIR_TWO_FRAME = "slalom_layer_1"
CHANNEL_PAIR_THREE_FRAME = "slalom_layer_2"

CHANNEL_PAIR_ONE_FRAME_CLUSTERED = "slalom_layer_0/clustered"
CHANNEL_PAIR_TWO_FRAME_CLUSTERED = "slalom_layer_1/clustered"
CHANNEL_PAIR_THREE_FRAME_CLUSTERED = "slalom_layer_2/clustered"

TRANSFORM_TIMEOUT_DURATION = 10.0

FIRST_VIEW = {"position_x": 0.0, "position_y": 0.0, "position_z": 0.8, "yaw": 0.0}
SECOND_VIEW = {"position_x": 0.0, "position_y": 0.0, "position_z": 0.8, "yaw": 0.0}
THIRD_VIEW = {"position_x": 0.0, "position_y": 0.0, "position_z": 0.8, "yaw": 0.0}
#########################################################################


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

    move_to_task = create_move_to_task_root()

    # TODO: Update the frame and pose to move to
    move_and_cluster_one = create_move_and_cluster_root(
        pose_stamped=create_stamped_pose(
            BASE_LINK_FRAME,
            position_x=FIRST_VIEW["position_x"],
            position_y=FIRST_VIEW["position_y"],
            position_z=FIRST_VIEW["position_z"],
            yaw=FIRST_VIEW["yaw"],
        ),
    )

    move_and_cluster_two = create_move_and_cluster_root(
        pose_stamped=create_stamped_pose(
            BASE_LINK_FRAME,
            position_x=SECOND_VIEW["position_x"],
            position_y=SECOND_VIEW["position_y"],
            position_z=SECOND_VIEW["position_z"],
            yaw=SECOND_VIEW["yaw"],
        ),
    )

    move_and_cluster_three = create_move_and_cluster_root(
        pose_stamped=create_stamped_pose(
            BASE_LINK_FRAME,
            position_x=THIRD_VIEW["position_x"],
            position_y=THIRD_VIEW["position_y"],
            position_z=THIRD_VIEW["position_z"],
            yaw=THIRD_VIEW["yaw"],
        ),
    )

    seq_move_and_cluster = py_trees.composites.Sequence(
        name="Move to different views and cluster",
        memory=True,
        children=[move_and_cluster_one, move_and_cluster_two, move_and_cluster_three],
    )

    # Initialize the number of missing transforms in blackboard to be used by DynamicSetBlackboard
    init_missing_transforms = py_trees.behaviours.SetBlackboardVariable(
        name="Initialize missing transforms",
        variable_name=fk("missing_transforms"),
        variable_value=0,
        overwrite=True,
    )

    # There is no need to read/write the transforms from the blackboard, but the below is just a means of confirmation
    check_transform_one = py_trees_ros.transforms.ToBlackboard(
        name="Write channel pair one transform",
        variable_name=fk("channel_pair_one_transform"),
        target_frame=CHANNEL_PAIR_ONE_FRAME_CLUSTERED,
        source_frame=BASE_LINK_FRAME,
        qos_profile=qos_profile_system_default,
    )

    check_transform_two = py_trees_ros.transforms.ToBlackboard(
        name="Write channel pair two transform",
        variable_name=fk("channel_pair_two_transform"),
        target_frame=CHANNEL_PAIR_TWO_FRAME_CLUSTERED,
        source_frame=BASE_LINK_FRAME,
        qos_profile=qos_profile_system_default,
    )

    check_transform_three = py_trees_ros.transforms.ToBlackboard(
        name="Write channel pair three transform",
        variable_name=fk("channel_pair_three_transform"),
        target_frame=CHANNEL_PAIR_THREE_FRAME_CLUSTERED,
        source_frame=BASE_LINK_FRAME,
        qos_profile=qos_profile_system_default,
    )

    update_missing_transforms_one = DynamicSetBlackboard(
        name="Update missing transforms one",
        key=fk("missing_transforms"),
        update_key=fk("missing_transforms"),
        overwrite=True,
        func=lambda x: x + 1,
    )

    update_missing_transforms_two = DynamicSetBlackboard(
        name="Update missing transforms two",
        key=fk("missing_transforms"),
        update_key=fk("missing_transforms"),
        overwrite=True,
        func=lambda x: x + 1,
    )

    update_missing_transforms_three = DynamicSetBlackboard(
        name="Update missing transforms three",
        key=fk("missing_transforms"),
        update_key=fk("missing_transforms"),
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
                child=check_transform_one,
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
                child=check_transform_two,
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
                child=check_transform_three,
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

    # Generate movemement options based on the number of missing transforms, generation done in compile time, execution done in runtime
    move_channel_one = create_channel_movement_root(0)
    move_channel_two = create_channel_movement_root(1)
    move_channel_three = create_channel_movement_root(2)

    check_missing_transforms_one = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check missing transforms one",
        check=py_trees.common.ComparisonExpression(
            variable=fk("missing_transforms"),
            value=0,
            operator=lambda x, y: operator.__eq__(x, y),
        ),
    )

    check_missing_transforms_two = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check missing transforms two",
        check=py_trees.common.ComparisonExpression(
            variable=fk("missing_transforms"),
            value=1,
            operator=lambda x, y: operator.__eq__(x, y),
        ),
    )

    check_missing_transforms_three = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check missing transforms three",
        check=py_trees.common.ComparisonExpression(
            variable=fk("missing_transforms"),
            value=2,
            operator=lambda x, y: operator.__eq__(x, y),
        ),
    )

    # Selector to choose the movement strategy based on the number of missing transforms
    select_movement_strategy = py_trees.composites.Selector(
        name="Select movement strategy",
        memory=True,
        children=[
            py_trees.composites.Sequence(
                name="No missing transforms",
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

    root.add_children(
        [
            move_to_task,
            seq_move_and_cluster,
            seq_check_transforms,
            select_movement_strategy,
        ]
    )

    return root
