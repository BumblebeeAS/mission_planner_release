#!/usr/bin/env python3

import py_trees
import py_trees.console as console
import py_trees_ros.trees
import rclpy

# from mission_planner_2.trees.auv.mother.mother import create_mother
from mission_planner_2.trees.auv.slalom.slalom import create_slalom_root


def main():
    rclpy.init(args=None)
    root = create_slalom_root()
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
            raise SystemExit
        elif tree.root.status == py_trees.common.Status.FAILURE:
            console.loginfo(console.red + "stopped on failure" + console.reset)
            raise SystemExit

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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
