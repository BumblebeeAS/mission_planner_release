#!/usr/bin/env python3

import py_trees
import py_trees.console as console
import py_trees_ros.trees
import rclpy

from mission_planner_2.trees.miniauv.tests.test_goto import (
    create_goto_test as tree_root,
)

# from mission_planner_2.trees.auv.tests.test_goto_nfrombb import (
#     create_test_multi_waypoint_root as tree_root,
# )
# from mission_planner_2.trees.auv.tests.test_convert_service import (
#     create_test_convert_service_root as tree_root,
# )
# from mission_planner_2.trees.auv.tests.test_preempt_controls import (
#     create_preempt_root as tree_root,
# )
# from mission_planner_2.trees.auv.tests.test_multi_cluster import (
#     create_test_multi_cluster_root as tree_root,
# )
# from mission_planner_2.trees.auv.tests.test_multi_waypoint import (
#     create_test_multi_waypoint_root as tree_root,
# )


def main():
    rclpy.init(args=None)
    root = tree_root()
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
            exit(0)

    tree.add_post_tick_handler(stop_on_success)
    tree.tick_tock(period_ms=100)
    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
