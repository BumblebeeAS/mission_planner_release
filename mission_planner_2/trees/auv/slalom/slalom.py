import py_trees
import py_trees_ros
from mission_planner_2.commons.namespace_utils import (
    generate_namespace,
    full_key_generator,
)
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.slalom.move_and_cluster import create_move_and_cluster_root
from mission_planner_2.trees.auv.slalom.move_to_task import create_move_to_task_root

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

########################## UPDATE CONSTANTS HERE #########################
BASE_LINK_FRAME = "auv4/base_link_ned"
CHANNEL_PAIR_ONE_FRAME = "channel_pair_one/yolo"
CHANNEL_PAIR_TWO_FRAME = "channel_pair_two/yolo"
CHANNEL_PAIR_THREE_FRAME = "channel_pair_three/yolo"

CHANNEL_PAIR_ONE_FRAME_CLUSTERED = "channel_pair_one/yolo/clustered"
CHANNEL_PAIR_TWO_FRAME_CLUSTERED = "channel_pair_two/yolo/clustered"
CHANNEL_PAIR_THREE_FRAME_CLUSTERED = "channel_pair_three/yolo/clustered"
#########################################################################

def create_slalom_root():
    """
    Create the root of the slalom tree.
    This tree will handle moving to the slalom task location,
    clustering the environment, and stabilizing the AUV.
    """
    root = py_trees.composites.Sequence(
        name="Slalom Task",
        memory=True,
    )

    move_to_task = create_move_to_task_root()

    move_and_cluster_one = create_move_and_cluster_root(
        pose_stamped=create_stamped_pose(BASE_LINK_FRAME, position_x=0.0, position_y=0.0, position_z=0.8, yaw=0.0),
        in_children=[CHANNEL_PAIR_ONE_FRAME, CHANNEL_PAIR_TWO_FRAME, CHANNEL_PAIR_THREE_FRAME],
        out_children=[CHANNEL_PAIR_ONE_FRAME_CLUSTERED, CHANNEL_PAIR_TWO_FRAME_CLUSTERED, CHANNEL_PAIR_THREE_FRAME_CLUSTERED],
    )

    move_and_cluster_two = create_move_and_cluster_root(
        pose_stamped=create_stamped_pose(BASE_LINK_FRAME, position_x=0.0, position_y=0.0, position_z=0.8, yaw=0.0),
        in_children=[CHANNEL_PAIR_ONE_FRAME, CHANNEL_PAIR_TWO_FRAME, CHANNEL_PAIR_THREE_FRAME],
        out_children=[CHANNEL_PAIR_ONE_FRAME_CLUSTERED, CHANNEL_PAIR_TWO_FRAME_CLUSTERED, CHANNEL_PAIR_THREE_FRAME_CLUSTERED],
    )

    move_and_cluster_three = create_move_and_cluster_root(
        pose_stamped=create_stamped_pose(BASE_LINK_FRAME, position_x=0.0, position_y=0.0, position_z=0.8, yaw=0.0),
        in_children=[CHANNEL_PAIR_ONE_FRAME, CHANNEL_PAIR_TWO_FRAME, CHANNEL_PAIR_THREE_FRAME],
        out_children=[CHANNEL_PAIR_ONE_FRAME_CLUSTERED, CHANNEL_PAIR_TWO_FRAME_CLUSTERED, CHANNEL_PAIR_THREE_FRAME_CLUSTERED],
    )

    seq_move_and_cluster = py_trees.composites.Sequence(
        name="Move to different views and cluster",
        memory=True,
        children=[
            move_and_cluster_one,
            move_and_cluster_two,
            move_and_cluster_three
        ]
    )

    move_to_channel_pair_one = goto.FromConstant(
        name="Goto Channel Pair One",
        parent_namespace=NAMESPACE,
        pose=create_stamped_pose(CHANNEL_PAIR_ONE_FRAME_CLUSTERED),
    )

    move_to_channel_pair_two = goto.FromConstant(
        name="Goto Channel Pair Two",
        parent_namespace=NAMESPACE,
        pose=create_stamped_pose(CHANNEL_PAIR_TWO_FRAME_CLUSTERED),
    )

    move_to_channel_pair_three = goto.FromConstant(
        name="Goto Channel Pair Three",
        parent_namespace=NAMESPACE,
        pose=create_stamped_pose(CHANNEL_PAIR_THREE_FRAME_CLUSTERED),
    )

    root.add_children(
        [
            move_to_task,
            seq_move_and_cluster
        ]
    )

    return root