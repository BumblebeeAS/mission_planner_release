import py_trees

from mission_planner_2.common.util.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.common.util.pose_utils import create_stamped_pose
from mission_planner_2.vehicles.auv.trees.robosub24.goto import goto

# Define a main namespace for the test
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

POSE_LIST_KEY = fk("pose_list")


def create_test_multi_waypoint_root():
    # Root sequence
    first_pose = create_stamped_pose(
        "auv4/base_link_ned", position_x=5.0, position_y=0.0, position_z=0.0
    )

    second_pose = create_stamped_pose(
        "auv4/base_link_ned", position_x=0.0, position_y=-5.0, position_z=0.0
    )

    third_pose = create_stamped_pose(
        "auv4/base_link_ned", position_x=-5.0, position_y=5.0, position_z=0.0
    )

    test_seq = py_trees.composites.Sequence(name="test sequence", memory=True)

    set_poses = py_trees.behaviours.SetBlackboardVariable(
        name="Set poses list",
        variable_name=POSE_LIST_KEY,
        variable_value=[first_pose, second_pose, third_pose],
        overwrite=True,
    )

    goto_n = goto.NFromBlackboard(
        name="Goto pose lists",
        pose_key=POSE_LIST_KEY,
    )

    test_seq.add_children([set_poses, goto_n])

    return test_seq
