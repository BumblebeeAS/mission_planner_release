#!/usr/bin/env python3

import py_trees
import py_trees.console as console
import py_trees_ros
import rclpy

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
from mission_planner_2.commons.hooks import stop_on_success_or_failure
from mission_planner_2.commons.node_registry import TreeNode
# from mission_planner_2.trees.auv.tests.test_goto import create_goto_test as tree_root

from mission_planner_2.trees.turtlesim.turtle_circle import create_turtle_circle_root as tree_root
from mission_planner_2.commons.led_management import create_led_tree

def main():
    rclpy.init(args=None)
    root = tree_root()
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree, node = create_led_tree(root)
    try:
        tree.setup(node=node, timeout=60.0)
    except:
        console.logerror(console.red + "failed to setup the tree" + console.reset)
        tree.shutdown()
        rclpy.shutdown()

    tree.add_post_tick_handler(stop_on_success_or_failure)
    tree.tick_tock(period_ms=100)
    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        console.loginfo(console.yellow + "interrupted" + console.reset)
    except SystemExit:
        console.loginfo(console.yellow + "exiting" + console.reset)
    except Exception as e:
        console.logfatal(
            console.red + "exception occurred: {}".format(e) + console.reset
        )
    finally:
        console.loginfo(console.reset + "cleaning up")
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
