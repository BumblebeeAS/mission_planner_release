import py_trees

from mission_planner_2.trees.auv.acoustics.order_by_ping import create_order_by_ping_root
from mission_planner_2.trees.auv.torpedo.torpedo import create_torpedo_root
from mission_planner_2.trees.auv.octagon.octagon import create_octagon_root


def create_acoustics_root(move_func) -> py_trees.behaviour.Behaviour:

    def acoustic_octagon() -> py_trees.behaviour.Behaviour:
        
        seq_acoustic_octagon = py_trees.composites.Sequence(
            name="Sequence acoustic octagon",
            memory=True
        )

        seq_acoustic_octagon.add_children(
            children=[
                move_func("acoustic_start", "octagon"),
                create_octagon_root(),
                move_func("octagon", "acoustic_start")
            ]
        )

        return seq_acoustic_octagon

    def acoustic_torpedo() -> py_trees.behaviour.Behaviour:

        seq_acoustic_torpedo = py_trees.composites.Sequence(
            name="Sequence acoustic torpedo",
            memory=True
        )

        seq_acoustic_torpedo.add_children(
            children=[
                move_func("acoustic_start", "torpedo_start"),
                create_torpedo_root(),
                move_func("torpedo_start", "acoustic_start")
            ]
        )

        return seq_acoustic_torpedo

    return create_order_by_ping_root(
        acoustic_octagon,
        acoustic_torpedo
    )
