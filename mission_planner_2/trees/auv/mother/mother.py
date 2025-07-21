import py_trees

from mission_planner_2.commons.pose_utils import compute_start_to_end_vector
from mission_planner_2.trees.auv.bins.bins import create_bin_root
from mission_planner_2.trees.auv.bins.move_to_task import create_move_to_bin_task_root
from mission_planner_2.trees.auv.gate.gate import create_gate_root
from mission_planner_2.trees.auv.gate.move_to_task import create_move_to_gate_task_root
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
WORLD_KEY = "/global/world"


MAP_NED_COORDS_GATE_START = {
    "x": 2.0,
    "y": 0.0,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
MAP_NED_COORDS_GATE_END = {  # TODO: MUST TUNE
    "x": 2.0,
    "y": 0.0,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
MAP_NED_COORDS_SLALOM_START = {
    "x": 7.0,
    "y": -1.0,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
MAP_NED_COORDS_SLALOM_END = {  # TODO: MUST TUNE
    "x": 7.0,
    "y": -1.0,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
MAP_NED_COORDS_BIN_START = {
    "x": 16.0,
    "y": 1.5,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
MAP_NED_COORDS_BIN_END = {  # TODO: MUST TUNE
    "x": 7.0,
    "y": -1.0,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
MAP_NED_COORDS_TORPEDO_START = {
    "x": 18.5,
    "y": -1.0,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
MAP_NED_COORDS_TORPEDO_END = {
    "x": 18.5,
    "y": -1.0,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
MAP_NED_COORDS_OCTAGON = {
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

    set_world_frame = py_trees.behaviours.SetBlackboardVariable(
        name="Set World Frame",
        variable_name=WORLD_KEY,
        variable_value="world_ned",
        overwrite=True,
    )

    gate_root = create_gate_root()
    move_to_gate = create_move_to_gate_task_root(MAP_NED_COORDS_GATE_START)

    gate_to_slalom_vector = compute_start_to_end_vector(
        MAP_NED_COORDS_GATE_END, MAP_NED_COORDS_SLALOM_START
    )
    move_to_slalom = create_move_to_slalom_task_root(gate_to_slalom_vector)
    slalom_root = create_slalom_root()

    slalom_to_bin_vector = compute_start_to_end_vector(
        MAP_NED_COORDS_SLALOM_END, MAP_NED_COORDS_BIN_START
    )
    move_to_bin = create_move_to_bin_task_root(slalom_to_bin_vector)
    bin_root = create_bin_root()

    bin_to_torpedo_vector = compute_start_to_end_vector(
        MAP_NED_COORDS_BIN_END, MAP_NED_COORDS_TORPEDO_START
    )
    move_to_torpedo = create_move_to_torpedo_task_root(bin_to_torpedo_vector)
    torpedo_root = create_torpedo_root()

    torpedo_to_octagon_vector = compute_start_to_end_vector(
        MAP_NED_COORDS_TORPEDO_END, MAP_NED_COORDS_OCTAGON
    )
    move_to_octagon = create_move_to_octagon_task_root(torpedo_to_octagon_vector)
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
            set_world_frame,  # TODO: use multi set bb?
            # move_to_gate,
            # gate_root,
            # move_to_slalom,
            # slalom_root,
            # move_to_bin,
            # bin_root,
            # move_to_torpedo,
            # torpedo_root,
            # move_to_octagon,
            octagon_root,
            # return_root,
        ]
    )

    return root
