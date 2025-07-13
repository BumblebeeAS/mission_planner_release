import py_trees
from nav_msgs.msg import Odometry
from py_trees_ros import subscribers

from mission_planner_2.commons import is_within_threshold
from mission_planner_2.trees.miniauv.goto import goto
from mission_planner_2.trees.miniauv.goto.stationkeep import create_stationkeep_root


def create_goto_with_threshold_root():
    """Creates the goto sequence to move to a predefined constant"""
    # FIXME: make the BB keys meant to be read outside an arg in the method
    # right now all the keys are hardcoded and the SAME but goto_node exposes the BB keys
    input_pose_to_goto = "input_pose_to_goto"
    threshold_key = "threshold_key"

    root = py_trees.composites.Parallel(
        name="movement_seq",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )

    check_seq = py_trees.composites.Sequence(
        name="check_seq",
        memory=False,
    )

    odom_sub = subscribers.ToBlackboard(
        name="odom_sub",
        topic_name="/orca4/odom_ned",
        topic_type=Odometry,
        qos_profile=10,
        blackboard_variables={"odom": "pose"},
    )

    check_selector = py_trees.composites.Selector(
        name="check_selector",
        memory=True,
    )

    check_within_threshold = is_within_threshold.FromBlackboard(
        name="check_within_threshold",
        key_pose="odom",
        key_target_pose=input_pose_to_goto,
        key_threshold=threshold_key,
    )

    goton = goto.FromBlackboard(
        name="goto_node",
        pose_key=input_pose_to_goto,
    )

    stationkeep = create_stationkeep_root()

    check_selector.add_children(
        [
            py_trees.decorators.Inverter(
                name="inverter",
                child=check_within_threshold,
            ),
            stationkeep,
        ]
    )

    check_seq.add_children([odom_sub, check_selector])

    root.add_children([check_seq, goton])

    return root
