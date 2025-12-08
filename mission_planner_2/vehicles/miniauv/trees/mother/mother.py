import py_trees

from mission_planner_2.vehicles.miniauv.trees.goto import (
    create_goto_with_threshold_root,
)

IS_LEFT_KEY = "/global/is_left_side"  # Global key for left option or not
BASE_LINK_KEY = "/global/base_link"


def create_mother():
    root = py_trees.composites.Sequence(
        name="mother",
        memory=True,
    )

    set_base_link_frame = py_trees.behaviours.SetBlackboardVariable(
        name="Set Base Link Frame",
        variable_name=BASE_LINK_KEY,
        variable_value="orca4_ned",
        overwrite=True,
    )

    # TODO: PURELY FOR TESTING
    set_is_left = py_trees.behaviours.SetBlackboardVariable(
        name="Set is left for test",
        variable_name=IS_LEFT_KEY,
        variable_value=True,
        overwrite=True,
    )

    root.add_children(
        [
            set_base_link_frame,
        ]
    )

    return root
