# mission_planner_2

## About
New BumblebeeAS mission planner stack using py_trees.

Refer to the [notion page](https://www.notion.so/nusbbas/Mission-Planner-2-1683cacaefa180c28574f8291d6e2977) for documentation to get started. The development guide is at the bottom of the documentation.

## Organization
Convenience API is located in `mission_planner_2/`.

Trees are written under `mission_planner_2/trees/<optional_subdirs>`.

Main executables are written under `scripts`.

## Usage
To create an executable for some mission, write a new ROS executable under `scripts` with the "execute once" template as given below. Replace the root with your behavior tree's root.

```python
#!/usr/bin/env python3
import py_trees
import py_trees.console as console
import py_trees_ros.trees

import rclpy

from mission_planner_2.trees.foo import bar

def main():
    rclpy.init(args=None)
    root = bar() # replace this
    py_trees.logging.level = py_trees.logging.Level.DEBUG
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

if __name__ == "__main__":
    main()

```
