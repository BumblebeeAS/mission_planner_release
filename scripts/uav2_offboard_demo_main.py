#!/usr/bin/env python3
import traceback

import py_trees
import py_trees.console as console
import rclpy

from mission_planner_2.common.core.bumble_tree import BumbleTree
from mission_planner_2.common.core.hooks import stop_on_success_or_failure
from mission_planner_2.common.core.visitors import LoggingSnapshotVisitor
from mission_planner_2.vehicles.uav2.config.node_registry import TreeNode
from mission_planner_2.vehicles.uav2.trees.offboard_demo.offboard_demo import (
    create_offboard_demo_root,
)


def main():
    rclpy.init(args=None)
    root = create_offboard_demo_root()
    py_trees.logging.level = py_trees.logging.Level.INFO
    tree = BumbleTree(root=root)
    node = TreeNode("uav2_offboard_demo")

    log_visitor = LoggingSnapshotVisitor(
        node=node,
        display_only_visited_behaviours=True,
        display_blackboard=False,
        display_activity_stream=True,
    )
    tree.add_visitor(log_visitor)

    try:
        tree.setup(node=node, timeout=600.0)
    except Exception:
        console.logerror(console.red + "failed to setup the tree" + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        return

    tree.add_post_tick_handler(stop_on_success_or_failure)

    tree.tick_tock(period_ms=100)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        console.loginfo(console.yellow + "interrupted" + console.reset)
    except SystemExit:
        console.loginfo(console.yellow + "mission finished — exiting" + console.reset)
    except Exception:
        console.logfatal(console.red + traceback.format_exc() + console.reset)
    finally:
        console.loginfo(console.reset + "cleaning up")
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
