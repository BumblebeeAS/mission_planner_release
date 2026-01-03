#!/usr/bin/env python3
# adapted from mission_planner_2/scripts/uav2_main.py 02/11/26

import traceback

import py_trees
import py_trees.console as console
import rclpy

from mission_planner_2.common.core.bumble_tree import BumbleTree
from mission_planner_2.common.core.hooks import stop_on_success_or_failure
from mission_planner_2.vehicles.auv.config.node_registry import AUVTreeNode
from mission_planner_2.common.core.visitors import LoggingSnapshotVisitor

from mission_planner_2.common.util.pose_utils import (
    create_stamped_pose,
)
from mission_planner_2.vehicles.auv.trees.robosub24.goto import goto

######################### UPDATE CONSTANTS HERE #########################
AUV4_FRAME_ID = "auv4/base_link_ned"

X_START = 2.5
Y_START = 0.0
Z_DEFAULT = -1.5 # we don't expect to move in Z for prequali
YAW_START = 0.0

# attemp using AUV4_FRAME_ID
# STARTING_POINT = create_stamped_pose( #TODO idk if it is supposed to be "map"
#     frame_id="map", position_x=X_START, position_y=Y_START, position_z=Z_DEFAULT,yaw=YAW_START
# )

# PAST_GATE_POSE_1 = create_stamped_pose(
#     frame_id=AUV4_FRAME_ID, position_x=3.0, position_y=0.0, position_z=Z_DEFAULT, yaw=0.0
# )

# PILLAIR_POSE_1 = create_stamped_pose(
#     frame_id=AUV4_FRAME_ID, position_x=9.5, position_y=-1.3, position_z=Z_DEFAULT, yaw=0.0
# )

# PILLAIR_POSE_2 = create_stamped_pose(
#     frame_id=AUV4_FRAME_ID, position_x=1.0, position_y=1.3, position_z=Z_DEFAULT, yaw=-90.0
# )

# PILLAIR_POSE_3 = create_stamped_pose(
#     frame_id=AUV4_FRAME_ID, position_x=1.0, position_y=1.3, position_z=Z_DEFAULT, yaw=-90.0
# )

# PAST_GATE_POSE_2 = create_stamped_pose(
#     frame_id=AUV4_FRAME_ID, position_x=9.5, position_y=-1.3, position_z=Z_DEFAULT, yaw=0.0
# )

# BACK_TO_STARTING_POINT = create_stamped_pose(
#     frame_id="map", position_x=X_START, position_y=Y_START, position_z=Z_DEFAULT,yaw=(YAW_START + 180.0)
# )

# Attempt using "map" frame
STARTING_POINT = create_stamped_pose( #TODO idk if it is supposed to be "map"
    frame_id="map", position_x=X_START, position_y=Y_START, position_z=Z_DEFAULT,yaw=YAW_START
)

PAST_GATE_POSE_1 = create_stamped_pose(
    frame_id="map", position_x=3.5, position_y=0.0, position_z=Z_DEFAULT, yaw=0.0
)

PILLAIR_POSE_1 = create_stamped_pose(
    frame_id="map", position_x=13.0, position_y=-1.3, position_z=Z_DEFAULT, yaw=0.0
)

PILLAIR_POSE_2 = create_stamped_pose(
    frame_id="map", position_x=14.0, position_y=0.0, position_z=Z_DEFAULT, yaw=-90.0
)

PILLAIR_POSE_3 = create_stamped_pose(
    frame_id="map", position_x=13.0, position_y=1.3, position_z=Z_DEFAULT, yaw=-180.0
)

PAST_GATE_POSE_2 = create_stamped_pose(
    frame_id="map", position_x=3.5, position_y=0.0, position_z=Z_DEFAULT, yaw=180.0
)

BACK_TO_STARTING_POINT = create_stamped_pose(
    frame_id="map", position_x=X_START, position_y=Y_START, position_z=Z_DEFAULT,yaw=180.0
)
    
#########################################################################


def create_mother():
    seq_prequali_root = py_trees.composites.Sequence(
        name="AUV4 Prequali Root", memory=True
    )

    waypt_1 = goto.FromConstant(
        name="Start Point",
        pose=STARTING_POINT,
    )

    waypt_2 = goto.FromConstant(
        name="Past Gate Pose 1",
        pose=PAST_GATE_POSE_1,
    )
    
    waypt_3 = goto.FromConstant(
        name="Pillar Pose 1",
        pose=PILLAIR_POSE_1,
    )
    
    waypt_4 = goto.FromConstant(
        name="Pillar Pose 2",
        pose=PILLAIR_POSE_2,
    )
    
    waypt_5 = goto.FromConstant(
        name="Pillar Pose 3",
        pose=PILLAIR_POSE_3,
    )
    
    waypt_6 = goto.FromConstant(
        name="Past Gate Pose 2",
        pose=PAST_GATE_POSE_2,
    )
    
    waypt_7 = goto.FromConstant(
        name="Back to Starting Point",
        pose=BACK_TO_STARTING_POINT,
    )

    seq_prequali_root.add_children([
        waypt_1,
        waypt_2,
        waypt_3,
        waypt_4,
        waypt_5,
        waypt_6,
        waypt_7,
    ])

    return seq_prequali_root



def main():
    rclpy.init(args=None)
    root = create_mother()
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree = BumbleTree(root=root)
    node = AUVTreeNode()

    ####### Add visitors #######
    log_visitor = LoggingSnapshotVisitor(
        node=node,
        display_only_visited_behaviours=True,
        display_blackboard=False,
        display_activity_stream=True,
    )
    tree.add_visitor(log_visitor)
    ############################

    try:
        tree.setup(node=node, timeout=600.0)
    except:
        console.logerror(console.red + "failed to setup the tree" + console.reset)
        tree.shutdown()
        rclpy.shutdown()

    ###### Add post-tick handlers ######
    tree.add_post_tick_handler(stop_on_success_or_failure)
    ###################################

    tree.tick_tock(period_ms=100)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        console.loginfo(console.yellow + "interrupted" + console.reset)
    except SystemExit:
        console.loginfo(console.yellow + "exiting" + console.reset)
    except Exception:
        console.logfatal(console.red + traceback.format_exc() + console.reset)
    finally:
        console.loginfo(console.reset + "cleaning up")
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
