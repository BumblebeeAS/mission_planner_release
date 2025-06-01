import operator

import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mission_planner_2.commons import cache_tf
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
)
from mission_planner_2.trees.auv.goto import goto

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

GATE_LEFT_POSE_KEY = "gate_left_pose"
GATE_RIGHT_POSE_KEY = "gate_right_pose"


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

    save_tf_left = cache_tf.ToBlackboard(
        name="Save TF Left",
        variable_name=fk(GATE_LEFT_POSE_KEY),
        start="auv4/gate/centre",
        end="auv4/gate/left",
        qos_profile=qos_profile_system_default,
    )

    save_tf_right = cache_tf.ToBlackboard(
        name="Save TF Right",
        variable_name=fk(GATE_RIGHT_POSE_KEY),
        start="auv4/gate/centre",
        end="auv4/gate/right",
        qos_profile=qos_profile_system_default,
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

    get_gate_orientation = py_trees_ros.subscribers.ToBlackboard(
        name="Get gate orientation",
        topic_name="/auv4/gate/shark_fish",
        topic_type=String,
        qos_profile=qos_profile_system_default,
        blackboard_variables={fk("gate_orientation"): None},
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

    """
    For sim.

    gate_init_pose = create_stamped_pose("world_ned", 5.98, 2.48, 1.16, 0.0, 0.0, -90)

    Publish the following tf to mock the gate detection:
    ros2 run tf2_ros static_transform_publisher 7 0 1.5 -1.57 0 0 world_ned auv4/gate
    """

    # TODO: move out into separate file; see torpedo.
    gate_init_pose = create_stamped_pose("world_ned", position_z=0.40)

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

    forward_pose = create_stamped_pose("auv4/base_link_ned", position_x=3.0)
    move_pass_gate = goto.FromConstant("Pass through gate", NAMESPACE, forward_pose)

    try_left_side.add_children(children=[check_is_left, move_to_gate_before_left])
    select_gate_side.add_children(children=[try_left_side, move_to_gate_before_right])

    cluster = py_trees_ros.action_clients.FromConstant(
        name="cluster_action",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_parent="auv4/front_cam_optical",
            in_child="gate",
            out_child="gate/clustered",
            duration=30,
            use_cache=False,
        ),
    )

    root.add_children(
        children=[
            move_towards_gate,
            cluster,
            save_gate_tfs,  # Save TFs before moving to see pictures
            move_to_see_pictures,
            py_trees.timers.Timer("Wait to stabilize", 20.0),
            get_is_fish,
            get_gate_orientation,
            select_gate_side,
            py_trees.timers.Timer("Wait to stabilize", 10.0),
            move_pass_gate,
        ]
    )

    return root
