import py_trees

from mission_planner_2.trees.uav2.tins.tins import create_helipad_root


def create_mother():
    root = py_trees.composites.Sequence(
        name="mother",
        memory=True,
    )

    helipad_root = create_helipad_root()

    root.add_children([helipad_root])

    return root
