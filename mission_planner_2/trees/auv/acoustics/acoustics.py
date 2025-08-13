import py_trees

from mission_planner_2.trees.auv.acoustics.order_by_ping import (
    create_order_by_ping_root,
)
from mission_planner_2.trees.auv.octagon.octagon import create_octagon_root
from mission_planner_2.trees.auv.torpedo.torpedo import create_torpedo_root

_TORPEDO_START = "torpedo_start"
_OCTAGON_START = "octagon"
_ACOUSTIC_START = "acoustic_start"
_TORPEDO = "torpedo"

# move to torpedo use torpedo_start move away from torpedo use torpedo


def create_acoustics_root(move_func, timeout) -> py_trees.behaviour.Behaviour:
    def acoustic_octagon() -> py_trees.behaviour.Behaviour:
        seq_acoustic_octagon = py_trees.composites.Sequence(
            name="Sequence acoustic octagon", memory=True
        )

        seq_acoustic_octagon.add_children(
            children=[
                move_func(_ACOUSTIC_START, _OCTAGON_START),
                create_octagon_root(),
                move_func(_OCTAGON_START, _ACOUSTIC_START),
            ]
        )

        return seq_acoustic_octagon

    def acoustic_torpedo() -> py_trees.behaviour.Behaviour:
        seq_acoustic_torpedo = py_trees.composites.Sequence(
            name="Sequence acoustic torpedo", memory=True
        )

        seq_acoustic_torpedo.add_children(
            children=[
                move_func(_ACOUSTIC_START, _TORPEDO_START),
                create_torpedo_root(),
                move_func(_TORPEDO, _ACOUSTIC_START),
            ]
        )

        return seq_acoustic_torpedo

    return create_order_by_ping_root(
        acoustic_octagon,
        acoustic_torpedo,
        timeout=timeout,
    )
