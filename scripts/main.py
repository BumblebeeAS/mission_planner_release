#!/usr/bin/env python3

import py_trees
import py_trees.console as console
import py_trees_ros.trees
import rclpy

# from mission_planner_2.trees.auv.bins.bins import create_bin_root as tree_root
from mission_planner_2.trees.auv.torpedo.torpedo import create_torpedo_root as tree_root

# from mission_planner_2.trees.auv.gate.gate import create_gate_root as tree_root


def main():
    rclpy.init(args=None)
    root = tree_root()
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    try:
        tree.setup(timeout=15.0)
    except:
        console.logerror(console.red + "failed to setup the tree" + console.reset)
        tree.shutdown()
        rclpy.shutdown()

    def stop_on_success_or_failure(tree):
        if tree.root.status == py_trees.common.Status.SUCCESS:
            console.loginfo(console.green + "completed one execution" + console.reset)
            raise KeyboardInterrupt
        elif tree.root.status == py_trees.common.Status.FAILURE:
            console.loginfo(console.red + "stopped on failure" + console.reset)
            raise KeyboardInterrupt

    tree.add_post_tick_handler(stop_on_success_or_failure)
    tree.tick_tock(period_ms=100)
    try:
        rclpy.spin(tree.node)
    except Exception as e:
        console.logerror(
            console.red + "exception occurred: {}".format(e) + console.reset
        )
        console.loginfo("stopping")
    finally:
        console.loginfo(console.reset)
        tree.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
