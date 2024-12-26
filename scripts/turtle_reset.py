#!/usr/bin/env python3
import mission_planner_2
import py_trees
import py_trees_ros
import py_trees.console as console
from std_srvs.srv import Empty, Trigger
import rclpy
import sys

def generate_reset_tree():
    reset_node = mission_planner_2.service_clients.FromConstant(
        name="reset_node",
        service_type=Empty,
        service_name="/reset",
        service_request=Empty.Request(),
    )

    root = py_trees.composites.Sequence("root", True, [reset_node])
    return root

def main():
    rclpy.init(args=None)
    root = generate_reset_tree()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
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

if name == "main":
    main()
