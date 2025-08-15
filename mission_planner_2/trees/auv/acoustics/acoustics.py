import py_trees

from mission_planner_2.trees.auv.acoustics.order_by_ping import (
    create_order_by_ping_root,
)

_TORPEDO_START = "torpedo_start"
_OCTAGON_START = "octagon"
_ACOUSTIC_END = "acoustic_end"
_TORPEDO = "torpedo"
_TORPEDO_POST = "torpedo_post"


PARTITION_OFFSET = 0
CONFIDENCE_THREHOLD = -1.0
PING_TOPIC = "/sensors/ping"


def _create_octagon_torpedo_root(
    move_func,
    octagon_root,
    torpedo_root,
    goto_depth: float,
    specified_heading_octagon: bool = True,
    specified_heading_torpedo: bool = True,
):
    root = py_trees.composites.Sequence(
        name="Octagon - torpedo seq",
        memory=True,
    )

    move_to_octagon = move_func(
        start_coords=_ACOUSTIC_END,
        end_coords=_OCTAGON_START,
        goto_depth=goto_depth,
        specified_heading=specified_heading_octagon,
    )

    octagon = octagon_root()
    force_succeed_octagon = py_trees.decorators.FailureIsSuccess(
        name="Force succeed octagon",
        child=octagon,
    )

    move_to_torpedo_post = move_func(
        start_coords=_OCTAGON_START,
        end_coords=_TORPEDO_POST,
        goto_depth=goto_depth,
        specified_heading=specified_heading_torpedo,
    )

    move_to_torpedo_start = move_func(
        start_coords=_TORPEDO_POST,
        end_coords=_TORPEDO_START,
        goto_depth=goto_depth,
        specified_heading=specified_heading_torpedo,
    )

    torpedo = torpedo_root()
    force_succeed_torpedo = py_trees.decorators.FailureIsSuccess(
        name="Force succeed torpedo",
        child=torpedo,
    )

    root.add_children(
        [
            move_to_octagon,
            force_succeed_octagon,
            move_to_torpedo_post,
            move_to_torpedo_start,
            force_succeed_torpedo,
        ]
    )

    return root


def _create_torpedo_octagon_root(
    move_func,
    octagon_root,
    torpedo_root,
    goto_depth: float,
    specified_heading_octagon: bool = True,
    specified_heading_torpedo: bool = True,
):
    root = py_trees.composites.Sequence(
        name="Octagon - torpedo seq",
        memory=True,
    )

    move_to_torpedo_post = move_func(
        start_coords=_ACOUSTIC_END,
        end_coords=_TORPEDO_POST,
        goto_depth=goto_depth,
        specified_heading=specified_heading_torpedo,
    )

    move_to_torpedo_start = move_func(
        start_coords=_TORPEDO_POST,
        end_coords=_TORPEDO_START,
        goto_depth=goto_depth,
        specified_heading=specified_heading_torpedo,
    )

    torpedo = torpedo_root()
    force_succeed_torpedo = py_trees.decorators.FailureIsSuccess(
        name="Force succeed torpedo",
        child=torpedo,
    )

    move_to_torpedo_post_octagon = move_func(
        start_coords=_TORPEDO,
        end_coords=_TORPEDO_POST,
        goto_depth=goto_depth,
        specified_heading=specified_heading_octagon,
    )

    move_to_post_octagon = move_func(
        start_coords=_TORPEDO_POST,
        end_coords=_OCTAGON_START,
        goto_depth=goto_depth,
        specified_heading=specified_heading_octagon,
    )

    octagon = octagon_root()
    force_succeed_octagon = py_trees.decorators.FailureIsSuccess(
        name="Force succeed octagon",
        child=octagon,
    )

    root.add_children(
        [
            move_to_torpedo_post,
            move_to_torpedo_start,
            force_succeed_torpedo,
            move_to_torpedo_post_octagon,
            move_to_post_octagon,
            force_succeed_octagon,
        ]
    )

    return root


def create_acoustics_root(
    move_func,
    octagon_root,
    torpedo_root,
    timeout: float,
    is_octagon_on_right: bool,
    goto_depth: float = 0.3,
    specified_heading_torpedo: bool = True,
    specified_heading_octagon: bool = True,
) -> py_trees.behaviour.Behaviour:
    octagon_on_left_adjustment = 0 if is_octagon_on_right else 180

    seq_octagon_torpedo = _create_octagon_torpedo_root(
        move_func=move_func,
        octagon_root=octagon_root,
        torpedo_root=torpedo_root,
        goto_depth=goto_depth,
        specified_heading_octagon=specified_heading_octagon,
        specified_heading_torpedo=specified_heading_torpedo,
    )

    seq_torpedo_octagon = _create_torpedo_octagon_root(
        move_func=move_func,
        octagon_root=octagon_root,
        torpedo_root=torpedo_root,
        goto_depth=goto_depth,
        specified_heading_octagon=specified_heading_octagon,
        specified_heading_torpedo=specified_heading_torpedo,
    )

    return create_order_by_ping_root(
        octagon_torpedo_execution=seq_octagon_torpedo,
        torpedo_octagon_execution=seq_torpedo_octagon,
        ping_topic=PING_TOPIC,
        timeout=timeout,
        confidence_threshold=CONFIDENCE_THREHOLD,
        partition_angle_offset=PARTITION_OFFSET - octagon_on_left_adjustment,
    )
