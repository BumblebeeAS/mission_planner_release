import py_trees
from geometry_msgs.msg import PoseStamped

from mission_planner_2.trees.auv.goto import goto_node
from mission_planner_2.trees.auv.goto.goto import create_goto_root


def create_move_to_task_root() -> py_trees.behaviour.Behaviour:
    """
    Create the root of the bin tree.
    """
    root = py_trees.composites.Sequence(
        name="Move to Task Root",
        memory=True,
    )

    # TODO: figure out the move to board subtree

    return root
