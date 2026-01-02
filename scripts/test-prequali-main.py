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
#########################################################################


def create_mother():
    seq_prequali_root = py_trees.composites.Sequence(
        name="AUV4 Prequali Root", memory=True
    )

    goto_forward = goto.FromConstant(
        name="Goto forward test",
        pose=create_stamped_pose(frame_id=AUV4_FRAME_ID, position_x=5.0),
    )

    seq_prequali_root.add_children(
        [
            goto_forward,
        ]
    )

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
