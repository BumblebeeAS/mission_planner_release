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

IS_LEFT_KEY = "/global/is_left_side"  # Global key for left option or not
BASE_LINK_KEY = "/global/base_link"

WORLD_NED_COORDS_SLALOM = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
WORLD_NED_COORDS_BIN = {
    "x": 6.0,
    "y": 0.0,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
WORLD_NED_COORDS_TORPEDO = {
    "x": 6.0,
    "y": 0.6,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
WORLD_NED_COORDS_OCTAGON = {
    "x": 6.0,
    "y": 1.2,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}


def create_mother():
    root = py_trees.composites.Sequence(
        name="mother",
        memory=True,
    )

    set_base_link_frame = py_trees.behaviours.SetBlackboardVariable(
        name="Set Base Link Frame",
        variable_name=BASE_LINK_KEY,
        variable_value="auv4/base_link_ned",
        overwrite=True,
    )

    gate_root = create_gate_root()

    move_to_slalom = create_move_to_slalom_task_root(WORLD_NED_COORDS_SLALOM)
    slalom_root = create_slalom_root()

    move_to_bin = create_move_to_bin_task_root(WORLD_NED_COORDS_BIN)
    bin_root = create_bin_root()

    move_to_torpedo = create_move_to_torpedo_task_root(WORLD_NED_COORDS_TORPEDO)
    torpedo_root = create_torpedo_root()

    move_to_octagon = create_move_to_octagon_task_root(WORLD_NED_COORDS_OCTAGON)
    octagon_root = create_octagon_root()

    # TODO: see if need a move to gate here to go closer to do the return task
    return_root = create_return_root()

    # TODO: PURELY FOR TESTING
    set_is_left = py_trees.behaviours.SetBlackboardVariable(
        name="Set is left for test",
        variable_name=IS_LEFT_KEY,
        variable_value=True,
        overwrite=True,
    )

    root.add_children(
        [
            set_is_left,
            set_base_link_frame,
            # gate_root,
            # move_to_slalom,
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
