import operator

import py_trees
import py_trees_ros.trees
from bb_controls_msgs.action import Locomotion

from mission_planner_2 import dynamic_set_blackboard, service_clients


def _gen_stationkeep_goal():
    goal_msg = Locomotion.Goal()

    goal_msg.move_rel = True
    goal_msg.depth_rel = True
    goal_msg.heading_rel = True
    try:
        goal_msg.depth_ctrl = Locomotion.Goal.DEPTH_MODE_DEPTH
    except Exception as e:
        print(e)
        goal_msg.depth_ctrl = 0

    goal_msg.specified_heading = False

    goal_msg.forward_setpoints = []
    goal_msg.sidemove_setpoints = []
    goal_msg.depth_setpoints = []
    goal_msg.heading_setpoints = []
    goal_msg.roll_setpoints = []
    goal_msg.pitch_setpoints = []
    goal_msg.altitude_setpoints = []

    return goal_msg


def create_stationkeep_root():
    """
    Calls locomotion action server to go to stationkeep at current position
    """

    stationkeep = py_trees_ros.action_clients.FromConstant(
        name="stationkeep",
        action_type=Locomotion,
        action_name="/auv4/controls",
        action_goal=_gen_stationkeep_goal(),
    )

    return stationkeep
