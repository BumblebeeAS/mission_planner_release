import operator
import py_trees

from std_msgs.msg import Bool

from mission_planner_2.commons.namespace_utils import full_key_generator
from mission_planner_2.commons.namespace_utils import generate_namespace
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto


NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)


def create_gate_root():
    """
    Create the root of the gate tree.

    Execution Flow:
    1. Move closer to the gate
    2. Enable detections
    3. Check if enable succeeded
    4. Move to gate pose
    5. Pass through gate
    6. Disable detections
    7. Disable detections succeeded
    """

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
        0.0,
        2.0,
        90.0,
        90.0,
        0.0
    )

    move_to_see_pictures = goto.FromConstant(
        "Move to pictures",
        NAMESPACE,
        pictures_pose
    )

    # it's a bit weird but its to simulate the possibility of service call
    # returning a ROS bool message rather than having a primitive bool
    set_gate_side = py_trees.behaviours.SetBlackboardVariable(
        name="Set gate side",
        variable_name=fk("is_left"),
        variable_value=Bool(data=True),
        overwrite=True
    )

    check_is_left = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Is left?",
        check=py_trees.common.ComparisonExpression(
            variable=fk("is_left"),
            value=True,
            operator=lambda x, y: operator.eq(x.data, y)
        ),
    )

    ##
    ## TODO: enable disable logic here
    ##

    #  TODO: determine init gate pose

    gate_init_pose = create_stamped_pose(
        "world_ned",
        0.0,
        0.0,
        0.75,
        0.0,
        0.0,
        0.0
    )

    # Publish the following tf to mock the gate detection
    # ros2 run tf2_ros static_transform_publisher 7 0 1.5 -1.57 0 0 world_ned auv4/gate

    # TODO: determine left and right offsets

    gate_before_left_pose = create_stamped_pose(
        "advay_please_remove_this",
        0.75,
        0.0,
        0.5,
        90.0,
        90.0,
        0.0
    )

    gate_before_right_pose = create_stamped_pose(
        "advay_please_remove_this",
        2.25,
        0.0,
        0.5,
        90.0,
        90.0,
        0.0
    )

    forward_pose = create_stamped_pose(
        "auv4/base_link_ned",
        1.5,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0
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
            py_trees.timers.Timer("Wait to stabilize", 5.0),
            move_to_see_pictures,
            py_trees.timers.Timer("Wait to stabilize", 5.0),
            set_gate_side,
            select_gate_side,
            py_trees.timers.Timer("Wait to stabilize", 5.0),
            move_pass_gate
        ]
    )

    return root
