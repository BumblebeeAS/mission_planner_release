from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto

# Define a main namespace for the test
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)


def create_test_multi_waypoint_root():
    # Root sequence
    first_pose = create_stamped_pose(
        "auv4/base_link_ned", position_x=5.0, position_y=0.0, position_z=0.0
    )

    second_pose = create_stamped_pose(
        "auv4/base_link_ned", position_x=0.0, position_y=-5.0, position_z=0.0
    )
    goto_test = goto.FromConstant(
        "first goto", pose=[first_pose, second_pose], specified_heading=False
    )

    return goto_test
