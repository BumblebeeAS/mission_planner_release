import py_trees

from mission_planner_2.trees.auv.goto import goto_node
from mission_planner_2.trees.auv.goto.stationkeep import create_stationkeep_root


# TODO: NOT tested yet
def create_cancel_goal_test() -> py_trees.common.Status:
    root = py_trees.composites.Parallel(
        name="preempt test",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )

    cancel_seq = py_trees.composites.Sequence(
        name="preempt",
        memory=True,
    )

    move_seq = py_trees.composites.Sequence(
        name="move_seq",
        memory=True,
    )

    timer = py_trees.timers.Timer(
        name="timer",
        duration=3.0,
    )

    set_pose_in_bb = py_trees.behaviours.SetBlackboardVariable(
        name="set_goto_1",
        variable_name="input_pose_to_goto",
        variable_value="",
        overwrite=True,
    )

    stationkeep = create_stationkeep_root()

    goto = goto_node.FromBlackboard(name="goto", pose_key="input_pose_to_goto")

    cancel_seq.add_children([timer, stationkeep])
    move_seq.add_children([set_pose_in_bb, goto])

    root.add_children(
        [
            cancel_seq,
            move_seq,
        ]
    )

    return root
