#!/usr/bin/env python3

import traceback

import py_trees
import py_trees.console as console
import rclpy

from mission_planner_2.commons.hooks import stop_on_success_or_failure
from mission_planner_2.commons.led_management import create_led_tree
from mission_planner_2.trees.auv.mother.mother import (
    create_mother,
    load_mission_coordinates,
)


def main():
    rclpy.init(args=None)
    coords = load_mission_coordinates()
    root = create_mother(coords)
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    tree, node = create_led_tree(root=root, display_only_visited_behaviours=True)
    try:
        tree.setup(node=node, timeout=60.0)
    except:
        console.logerror(console.red + "failed to setup the tree" + console.reset)
        tree.shutdown()
        rclpy.shutdown()

    tree.add_post_tick_handler(stop_on_success_or_failure)
    tree.tick_tock(period_ms=100)
    try:
        rclpy.spin(node)
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
