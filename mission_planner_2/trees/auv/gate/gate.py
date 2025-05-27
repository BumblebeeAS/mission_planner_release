import operator

import py_trees
import py_trees_ros
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String
from std_srvs.srv import Trigger
from transforms3d.euler import quat2euler

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

# TF save keys
GATE_LEFT_TF_KEY = "gate_left_tf"
GATE_RIGHT_TF_KEY = "gate_right_tf"
GATE_LEFT_POSE_KEY = "gate_left_pose"
GATE_RIGHT_POSE_KEY = "gate_right_pose"


def _tf_to_stamped_pose(tf):
    """
    Convert a TF to a StampedPose.
    """
    # convert quat to roll, pitch, yaw
    roll, pitch, yaw = quat2euler(
        [
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ],
    )

    return create_stamped_pose(
        frame_id=tf.header.frame_id,
        position_x=-tf.transform.translation.x,
        position_y=-tf.transform.translation.y,
        position_z=-tf.transform.translation.z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    )


def create_gate_root():
    root = py_trees.composites.Sequence(
        name="Gate Root",
        memory=True,
    )

    # Save TF sequence - executed before moving to see pictures
    save_gate_tfs = py_trees.composites.Parallel(
        name="Save Gate TFs",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )

    # Save transforms from base_link to gate left/right positions
    save_tf_left = py_trees_ros.transforms.ToBlackboard(
        name="Save TF Left",
        variable_name=fk(GATE_LEFT_TF_KEY),
        target_frame="auv4/gate/left",
        source_frame="auv4/base_link_ned",
        qos_profile=qos_profile_system_default,
    )

    save_tf_right = py_trees_ros.transforms.ToBlackboard(
        name="Save TF Right",
        variable_name=fk(GATE_RIGHT_TF_KEY),
        target_frame="auv4/gate/right",
        source_frame="auv4/base_link_ned",
        qos_profile=qos_profile_system_default,
    )

    # Convert saved TFs to poses
    reconstruct_left_pose = DynamicSetBlackboard(
        name="Reconstruct Left Pose",
        key=GATE_LEFT_TF_KEY,
        namespace=NAMESPACE,
        update_key=GATE_LEFT_POSE_KEY,
        overwrite=True,
        func=_tf_to_stamped_pose,
    )

    reconstruct_right_pose = DynamicSetBlackboard(
        name="Reconstruct Right Pose",
        key=GATE_RIGHT_TF_KEY,
        namespace=NAMESPACE,
        update_key=GATE_RIGHT_POSE_KEY,
        overwrite=True,
        func=_tf_to_stamped_pose,
    )

    save_gate_tfs.add_children([save_tf_left, save_tf_right])

    select_gate_side = py_trees.composites.Selector(
        name="Select gate side", memory=True
    )

    try_left_side = py_trees.composites.Sequence(name="Try left side", memory=True)

    get_is_fish = py_trees_ros.service_clients.FromConstant(
        name="Get is fish",
        service_type=Trigger,
        service_name="/auv4/choice/get_is_fish",
        service_request=Trigger.Request(),
        key_response=fk("is_fish"),
    )

    get_gate_orientation = py_trees.behaviours.SetBlackboardVariable(
        name="Get gate orientation",
        variable_name=fk("gate_orientation"),
        variable_value=String(data="fish_shark"),
        overwrite=True,
    )

    check_is_left = py_trees.behaviours.CheckBlackboardVariableValues(
        name="Is left?",
        checks=[
            py_trees.common.ComparisonExpression(
                variable=fk("is_fish"),
                value=True,
                operator=lambda x, y: operator.__eq__(x.success, y),
            ),
            py_trees.common.ComparisonExpression(
                variable=fk("gate_orientation"),
                value="fish_shark",
                operator=lambda x, y: operator.__eq__(x.data, y),
            ),
        ],
        operator=operator.__eq__,
    )

    gate_init_pose = create_stamped_pose("world_ned", 0.0, 0.0, 0.4, 0.0, 0.0, 0.0)

    # Publish the following tf to mock the gate detection
    # ros2 run tf2_ros static_transform_publisher 7 0 1.5 -1.57 0 0 world_ned auv4/gate
    move_towards_gate = goto.FromConstant(
        "Move towards gate", NAMESPACE, gate_init_pose
    )

    move_to_see_pictures = goto.FromConstant(
        "Move to pictures", NAMESPACE, create_stamped_pose("auv4/gate/centre")
    )

    # Now use saved poses instead of direct TF references
    move_to_gate_before_left = goto.FromBlackboard(
        name="Move to before left side",
        parent_namespace=NAMESPACE,
        pose_key=GATE_LEFT_POSE_KEY,
    )

    move_to_gate_before_right = goto.FromBlackboard(
        name="Move to before right side",
        parent_namespace=NAMESPACE,
        pose_key=GATE_RIGHT_POSE_KEY,
    )

    forward_pose = create_stamped_pose("auv4/base_link_ned", position_x=1.0)
    move_pass_gate = goto.FromConstant("Pass through gate", NAMESPACE, forward_pose)

    try_left_side.add_children(children=[check_is_left, move_to_gate_before_left])
    select_gate_side.add_children(children=[try_left_side, move_to_gate_before_right])

    root.add_children(
        children=[
            move_towards_gate,
            py_trees.timers.Timer("Wait to stabilize", 30.0),
            save_gate_tfs,  # Save TFs before moving to see pictures
            move_to_see_pictures,
            py_trees.timers.Timer("Wait to stabilize", 10.0),
            get_is_fish,
            get_gate_orientation,
            reconstruct_left_pose,
            reconstruct_right_pose,
            select_gate_side,  # Now uses saved poses
            py_trees.timers.Timer("Wait to stabilize", 10.0),
            move_pass_gate,
        ]
    )

    return root
