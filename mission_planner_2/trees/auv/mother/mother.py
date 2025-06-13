import py_trees

from mission_planner_2.trees.auv.bins.bins import create_bin_root
from mission_planner_2.trees.auv.bins.move_to_task import create_move_to_bin_task_root
from mission_planner_2.trees.auv.gate.gate import create_gate_root
from mission_planner_2.trees.auv.gate.return_home import create_return_root
from mission_planner_2.trees.auv.octagon.move_to_task import (
    create_move_to_octagon_task_root,
)
from mission_planner_2.trees.auv.octagon.octagon import create_octagon_root
from mission_planner_2.trees.auv.slalom.move_to_task import (
    create_move_to_slalom_task_root,
)
from mission_planner_2.trees.auv.slalom.slalom import create_slalom_root
from mission_planner_2.trees.auv.torpedo.move_to_task import (
    create_move_to_torpedo_task_root,
)
from mission_planner_2.trees.auv.torpedo.torpedo import create_torpedo_root


def create_mother():
    root = py_trees.composites.Sequence(
        name="mother",
        memory=True,
    )

    gate_root = create_gate_root()

    move_to_slalom = create_move_to_slalom_task_root()
    slalom_root = create_slalom_root()

    move_to_bin = create_move_to_bin_task_root()
    bin_root = create_bin_root()

    move_to_torpedo = create_move_to_torpedo_task_root()
    torpedo_root = create_torpedo_root()

    move_to_octagon = create_move_to_octagon_task_root()
    octagon_root = create_octagon_root()

    # TODO: see if need a move to gate here to go closer to do the return task
    return_root = create_return_root()

    root.add_children(
        [
            # gate_root,
            move_to_slalom,
            slalom_root,
            # move_to_bin,
            # bin_root,
            # move_to_torpedo,
            # torpedo_root,
            # move_to_octagon,
            # octagon_root,
            # return_root,
        ]
    )

    return root
