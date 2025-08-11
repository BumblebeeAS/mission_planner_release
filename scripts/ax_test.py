#!/usr/bin/env python3

import functools
import traceback

import py_trees
import py_trees.console as console
import rclpy

from mission_planner_2.commons.bumble_tree import BumbleTree
from mission_planner_2.commons.hooks import stop_on_success_or_failure
from mission_planner_2.commons.led_management import LedVisitor, led_handler
from mission_planner_2.commons.node_registry import TreeNode
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.acoustics.order_by_ping import create_order_by_ping_root
from mission_planner_2.trees.auv.goto import goto


def left_func() -> py_trees.behaviour.Behaviour:
    return goto.FromConstant(
        name="goto_left",
        pose=create_stamped_pose(
            "auv4/base_link_ned",
            yaw=-90.0
        )
    )

def right_func() -> py_trees.behaviour.Behaviour:
    return goto.FromConstant(
        name=create_stamped_pose(
            "auv4/base_link_ned",
            yaw=90.0
        )
    )

def main():
    rclpy.init(args=None)
    root = create_order_by_ping_root(
        right_func,
        left_func,
        confidence_threshold=5.0
    )
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree = BumbleTree(root=root)
    node = TreeNode()

    led_visitor = LedVisitor(full=True)
    tree.add_visitor(led_visitor)

    try:
        tree.setup(node=node, timeout=60.0)
    except:
        console.logerror(console.red + "failed to setup the tree" + console.reset)
        tree.shutdown()
        rclpy.shutdown()

    tree.add_post_tick_handler(stop_on_success_or_failure)
    tree.add_post_tick_handler(functools.partial(led_handler, led_visitor, node.led_publisher))

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
