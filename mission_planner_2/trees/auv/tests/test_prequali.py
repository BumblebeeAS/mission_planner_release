import csv
import operator

import py_trees
import py_trees_ros.trees
from bb_auv_msgs.srv import ResetPose
from bb_controls_msgs.action import Locomotion
from bb_controls_msgs.srv import Controller

from mission_planner_2 import dynamic_set_blackboard, service_clients

WAYPOINTS = []
OFFSET = []
CSV_FILEPATH = (
    "/home/bbauv4/workspaces/ros2_ws/src/controls_tests/controls_tests/quali.csv"
)


def _gen_goal(idx):
    goal_msg = Locomotion.Goal()

    # Set the required fields
    goal_msg.move_rel = False
    goal_msg.depth_rel = False
    goal_msg.heading_rel = False
    try:
        goal_msg.depth_ctrl = Locomotion.Goal.DEPTH_MODE_DEPTH
    except Exception as e:
        print(e)
        goal_msg.depth_ctrl = 0
    goal_msg.specified_heading = True

    # Set the setpoints
    goal_msg.forward_setpoints = [WAYPOINTS[idx][0] + OFFSET[0]]
    goal_msg.sidemove_setpoints = [WAYPOINTS[idx][1] + OFFSET[1]]
    goal_msg.depth_setpoints = [WAYPOINTS[idx][2] + OFFSET[2]]
    goal_msg.heading_setpoints = [WAYPOINTS[idx][3] + OFFSET[3]]

    # Lists that need to be populated but aren't used
    goal_msg.roll_setpoints = [0.0]
    goal_msg.pitch_setpoints = [0.0]
    goal_msg.altitude_setpoints = []

    return goal_msg


def _set_waypoints(csv_filepath):
    """
    Load waypoints from a CSV file.
    The first line is the offset, subsequent lines are waypoints.

    Args:
        csv_filepath: Path to the CSV file
    """
    global OFFSET, WAYPOINTS

    try:
        with open(csv_filepath, "r") as csvfile:
            reader = csv.reader(csvfile)
            for i, row in enumerate(reader):
                if len(row) != 4:
                    continue

                try:
                    x, y, z, yaw = map(float, row)
                    WAYPOINTS.append((x, y, z, yaw))
                except ValueError as e:
                    continue
    except Exception as e:
        return

    if not WAYPOINTS:
        return

    # Extract the offset (first line)
    OFFSET = WAYPOINTS[0]

    # remove the offset from the waypoints
    WAYPOINTS = WAYPOINTS[1:]


_set_waypoints(CSV_FILEPATH)


def create_pre_qual_root() -> py_trees.common.Status:
    reset_req = ResetPose.Request()
    reset_req.reset = True

    controller_req = Controller.Request()
    controller_req.enable = True
    controller_req.disable_altitude = False
    controller_req.pause = False

    main_seq = py_trees.composites.Sequence(
        name="main_seq",
        memory=True,
    )

    reset_pose_client = service_clients.FromConstant(
        name="reset_pose",
        service_type=ResetPose,
        service_name="/auv4/nav/reset_pose",
        service_request=reset_req,
    )

    enable_controller_client = service_clients.FromConstant(
        name="enable_controller",
        service_type=Controller,
        service_name="/auv4/controls/controller",
        service_request=controller_req,
    )

    movement_selector = py_trees.composites.Selector(
        name="movement_selector",
        memory=True,
    )

    init_idx = py_trees.behaviours.SetBlackboardVariable(
        name="set_waypoint_idx",
        variable_name="idx",
        variable_value=0,
        overwrite=True,
    )

    is_waypoints_done = py_trees.behaviours.CheckBlackboardVariableValue(
        name="check_idx_done",
        check=py_trees.common.ComparisonExpression(
            variable="idx",
            operator=operator.ge,
            value=len(WAYPOINTS),
        ),
    )

    movement_seq = py_trees.composites.Sequence(
        name="movement_seq",
        memory=True,
    )

    idx_updater = dynamic_set_blackboard.DynamicSetBlackboard(
        name="idx_updater",
        key="idx",
        update_key="idx",
        func=lambda i: i + 1,
        overwrite=True,
    )

    waypoint_updater = dynamic_set_blackboard.DynamicSetBlackboard(
        name="waypoint_updater",
        key="idx",
        update_key="waypoint_goal",
        func=lambda i: _gen_goal(i),
        overwrite=True,
    )

    send_action_goal = py_trees_ros.action_clients.FromBlackboard(
        name="action_client",
        action_type=Locomotion,
        action_name="/auv4/controls",
        key="waypoint_goal",
    )

    init_seq = py_trees.composites.Sequence(
        name="init",
        memory=True,
        children=[reset_pose_client, init_idx, enable_controller_client],
    )

    movement_seq.add_children([waypoint_updater, send_action_goal, idx_updater])
    movement_selector.add_children(
        [
            is_waypoints_done,
            py_trees.decorators.Inverter(
                name="invert_movement_stat", child=movement_seq
            ),
        ]
    )

    main_seq.add_children(
        [
            init_seq,
            py_trees.decorators.Retry(
                name="retry_until_success",
                child=movement_selector,
                num_failures=len(WAYPOINTS),
            ),
        ]
    )

    return main_seq
