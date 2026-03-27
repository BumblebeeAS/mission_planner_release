#!/usr/bin/env python3

import traceback

import py_trees
import py_trees.console as console
import rclpy

from mission_planner_2.common.core.bumble_tree import BumbleTree
from mission_planner_2.common.core.hooks import stop_on_success_or_failure
from mission_planner_2.common.core.visitors import LoggingSnapshotVisitor
from mission_planner_2.vehicles.auv.config.node_registry import AUVTreeNode
from mission_planner_2.vehicles.auv.trees.robosub26.mother.mother_quali import (
    create_mother,
)
from mission_planner_2.vehicles.auv.trees.shared.led_management import LedVisitor



def main():
    rclpy.init(args=None)
    root = create_mother()
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree = BumbleTree(root=root)
    # TODO: add the shared actions and services needed for the tree
    node = AUVTreeNode()

    ####### Add visitors #######
    led_visitor = LedVisitor(full=True)
    log_visitor = LoggingSnapshotVisitor(
        node=node,
        display_only_visited_behaviours=True,
        display_blackboard=False,
        display_activity_stream=True,
    )
    # tree.add_visitor(led_visitor)
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
    # tree.add_post_tick_handler(
    #     functools.partial(led_handler, led_visitor, node.led_publisher)
    # )
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
