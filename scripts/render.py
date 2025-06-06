#!/usr/bin/env python3

import py_trees

# from mission_planner_2.trees.auv.tests.test_preempt_controls import f
from mission_planner_2.trees.auv.slalom.slalom import create_slalom_root as f


def main():
    py_trees.display.render_dot_tree(
        f(),
        py_trees.common.VisibilityLevel.ALL,
        target_directory="/home/acheng289/workspaces/ros2_ws/src/mission_planner_2/images",
        with_blackboard_variables=False,
    )


if __name__ == "__main__":
    main()
