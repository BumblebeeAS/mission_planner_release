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


def create_preempt_root():
    # Root sequence
    first_pose = create_stamped_pose(
        "auv4/base_link_ned", position_x=10.0, position_y=0.0, position_z=0.0
    )

    second_pose = create_stamped_pose(
        "auv4/base_link_ned", position_x=0.0, position_y=-5.0, position_z=0.0
    )
    goto_second = goto.FromConstant("Second Goto", NAMESPACE, second_pose)
    goto_first = goto.FromConstant("First Goto", NAMESPACE, first_pose)

    timer_wait = py_trees.timers.Timer(
        "Wait Timer",
        4.0,  # Wait for 5 seconds
    )

    par = py_trees.composites.Parallel(
        name="fallback",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )
    fallback = py_trees.composites.Selector(name="fallbacl", memory=True)

    par.add_children(
        [
            goto_first,
            py_trees.decorators.SuccessIsFailure(name="invert", child=timer_wait),
        ]
    )

    fallback.add_children([par, goto_second])

    return fallback
