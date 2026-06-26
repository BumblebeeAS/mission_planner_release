import py_trees
from bb_uav_msgs.action import GoToPosition, Land, Takeoff

from mission_planner_2.common.core import shared_action_client
from mission_planner_2.vehicles.uav2.config.node_registry import UAV2SharedAction

# All positions are NED (z down-positive)
CLIMB_HEIGHT = 10.0
STANDOFF_NED = (-14.0, -25.0, -6.0)
RETURN_NED = (0.0, 0.0, -CLIMB_HEIGHT)
REACHED_TOL = 0.5
LAND_TIMEOUT = 30.0


def goto_goal(position) -> GoToPosition.Goal:
    x, y, z = position
    return GoToPosition.Goal(
        x=float(x),
        y=float(y),
        z=float(z),
        relative=False,
        x_threshold=REACHED_TOL,
        y_threshold=REACHED_TOL,
        z_threshold=REACHED_TOL,
    )


def create_offboard_demo_root() -> py_trees.behaviour.Behaviour:
    """Assemble the offboard demo as a memory sequence (each step runs to completion)."""
    root = py_trees.composites.Sequence(name="offboard_demo", memory=True)

    takeoff = shared_action_client.FromConstant(
        name="Takeoff",
        shared_action=UAV2SharedAction.TAKEOFF,
        action_goal=Takeoff.Goal(
            altitude=CLIMB_HEIGHT,
            x_threshold=REACHED_TOL,
            y_threshold=REACHED_TOL,
            z_threshold=REACHED_TOL,
        ),
    )

    goto_standoff = shared_action_client.FromConstant(
        name="Goto standoff",
        shared_action=UAV2SharedAction.GOTO,
        action_goal=goto_goal(STANDOFF_NED),
    )

    return_home = shared_action_client.FromConstant(
        name="Return home",
        shared_action=UAV2SharedAction.GOTO,
        action_goal=goto_goal(RETURN_NED),
    )

    land = shared_action_client.FromConstant(
        name="Land",
        shared_action=UAV2SharedAction.LAND,
        action_goal=Land.Goal(timeout=LAND_TIMEOUT),
    )

    root.add_children([takeoff, goto_standoff, return_home, land])

    return root
