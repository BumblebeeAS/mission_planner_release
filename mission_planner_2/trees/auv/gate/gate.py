import operator

import py_trees
import py_trees_ros
from std_msgs.msg import (
    Bool,
    String,
)
from std_srvs.srv import Trigger

from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)


def create_gate_root():

    DEPTH = 0.75

    root = py_trees.composites.Sequence(
        name="Gate Root",
        memory=True,
    )

    select_gate_side = py_trees.composites.Selector(
        name="Select gate side",
        memory=True
    )

    try_left_side = py_trees.composites.Sequence(
        name="Try left side",
        memory=True
    )

    pictures_pose = create_stamped_pose(
        "advay_please_remove_this",
        1.5,
        -DEPTH,
        2.0,
        90.0,
        90.0,
        0.0,
    )

    move_to_see_pictures = goto.FromConstant(
        "Move to pictures",
        NAMESPACE,
        pictures_pose
    )

    get_is_fish = py_trees_ros.service_clients.FromConstant(
        name="Get is fish",
        service_type=Trigger,
        service_name="/auv4/choice/is_fish",
        service_request=Trigger.Request(),
        key_response=fk("is_fish")
    )

    get_gate_orientation = py_trees.behaviours.SetBlackboardVariable(
        name="Get gate orientation",
        variable_name=fk("gate_orientation"),
        variable_value=String(data="fish_shark"),
        overwrite=True
    )

    check_is_left = py_trees.behaviours.CheckBlackboardVariableValues(
        name="Is left?",
        checks=[
            py_trees.common.ComparisonExpression(
                variable=fk("is_fish"),
                value=True,
                operator=lambda x, y: operator.__eq__(x.data, y)
            ),
            py_trees.common.ComparisonExpression(
                variable=fk("gate_orientation"),
                value="fish_shark",
                operator=lambda x, y: operator.__eq__(x.data, y)
            )
        ],
        operator=operator.__eq__,
    )

    ##
    ## TODO: enable disable logic here
    ##

    #  TODO: determine init gate pose

    gate_init_pose = create_stamped_pose("world_ned", 0.0, 0.0, 0.7, 0.0, 0.0, 0.0)

    # Publish the following tf to mock the gate detection
    # ros2 run tf2_ros static_transform_publisher 7 0 1.5 -1.57 0 0 world_ned auv4/gate

    gate_before_left_pose = create_stamped_pose(
        "advay_please_remove_this", 0.75, -DEPTH, 1.0, 90.0, 90.0, 0.0
    )

    gate_before_right_pose = create_stamped_pose(
        "advay_please_remove_this", 2.25, -DEPTH, 1.0, 90.0, 90.0, 0.0
    )

    forward_pose = create_stamped_pose(
        "auv4/base_link_ned", 1.5, 0.0, 0.0, 0.0, 0.0, 0.0
    )

    move_towards_gate = goto.FromConstant(
        "Move towards gate",
        NAMESPACE,
        gate_init_pose
    )

    move_to_gate_before_left = goto.FromConstant(
        "Move to before left side",
        NAMESPACE,
        gate_before_left_pose
    )

    move_to_gate_before_right = goto.FromConstant(
        "Move to before right side",
        NAMESPACE,
        gate_before_right_pose
    )

    move_pass_gate = goto.FromConstant(
        "Pass through gate",
        NAMESPACE,
        forward_pose
    )

    try_left_side.add_children(
        children=[
            check_is_left,
            move_to_gate_before_left
        ]
    )

    select_gate_side.add_children(
        children=[
            try_left_side,
            move_to_gate_before_right
        ]
    )

    root.add_children(
        children=[
            move_towards_gate,
            py_trees.timers.Timer("Wait to stabilize", 10.0),
            move_to_see_pictures,
            py_trees.timers.Timer("Wait to stabilize", 10.0),
            get_is_fish,
            get_gate_orientation,
            select_gate_side,
            py_trees.timers.Timer("Wait to stabilize", 10.0),
            move_pass_gate,
        ]
    )

    return root
