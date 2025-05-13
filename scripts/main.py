#!/usr/bin/env python3
import py_trees
import py_trees.console as console
import py_trees_ros.trees
import rclpy

from mission_planner_2.trees.auv.goto.goto import create_goto_root
from mission_planner_2.trees.auv.tests.test_prequali import create_pre_qual_root
from mission_planner_2.trees.turtlesim.turtle_circle import create_turtle_circle_root


def main():
    rclpy.init(args=None)
    root = create_goto_root()
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    try:
        tree.setup(timeout=15.0)
    except:
        console.logerror(console.red + "failed to setup the tree")
        tree.shutdown()
        rclpy.shutdown()

    def stop_on_success(tree):
        if tree.root.status == py_trees.common.Status.SUCCESS:
            console.loginfo(console.green + "completed one execution")
            tree.shutdown()
            rclpy.shutdown()

    tree.add_post_tick_handler(stop_on_success)
    tree.tick_tock(period_ms=100)
    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
