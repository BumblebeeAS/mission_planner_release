import py_trees
import py_trees_ros
from bb_auv_msgs.action import Grabber
from bb_sensor_msgs.msg import Ping
from rclpy.qos import qos_profile_system_default

from mission_planner_2.commons import shared_action_client
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.node_registry import SharedAction

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

_PING_RESPONSE_KEY = fk("ping")


def _create_get_ping_root(ping_topic: str):
    root = py_trees.composites.Sequence(
        name="Get ping seq",
        memory=True,
    )

    close_grabber = shared_action_client.FromConstant(
        name="Close grabber for pings",
        shared_action=SharedAction.GRABBER,
        action_goal=Grabber.Goal(
            command=5,
            tolerance=0,
            timeout_ms=5000,
        ),
    )

    force_succeed_close_grabber = py_trees.decorators.FailureIsSuccess(
        name="force succed close grabber",
        child=close_grabber,
    )

    sub_ping = py_trees_ros.subscribers.ToBlackboard(
        name="Subcribe to ping",
        topic_name=ping_topic,
        topic_type=Ping,
        qos_profile=qos_profile_system_default,
        blackboard_variables={_PING_RESPONSE_KEY: None},
    )

    open_grabber = shared_action_client.FromConstant(
        name="Open grabber for pings",
        shared_action=SharedAction.GRABBER,
        action_goal=Grabber.Goal(
            command=65535,
            tolerance=0,
            timeout_ms=5000,
        ),
    )

    force_succeed_open_grabber = py_trees.decorators.FailureIsSuccess(
        name="force succeed open grabber",
        child=open_grabber,
    )

    root.add_children(
        [
            force_succeed_close_grabber,
            sub_ping,
            force_succeed_open_grabber,
        ]
    )

    return root


def _create_ping_check_root(
    confidence_threshold: float = -1.0,
    partition_angle_offset: int = 0,
):
    root = py_trees.composites.Sequence(
        name="Check ping seq",
        memory=True,
    )

    check_ping_confidence = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check ping confidence",
        check=py_trees.common.ComparisonExpression(
            variable=_PING_RESPONSE_KEY,
            value=confidence_threshold,
            operator=lambda x, y: x.confidence >= y,
        ),
    )

    check_ping_direction = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check ping direction",
        check=py_trees.common.ComparisonExpression(
            variable=_PING_RESPONSE_KEY,
            value=180,
            operator=lambda x, y: (x.doa_deg - partition_angle_offset) % 360 < y,
        ),
    )

    root.add_children(
        [
            check_ping_confidence,
            check_ping_confidence,
        ]
    )

    return root


def create_order_by_ping_root(
    octagon_torpedo_execution: py_trees.behaviour.Behaviour,
    torpedo_octagon_execution: py_trees.behaviour.Behaviour,
    ping_topic: str = "/sensors/ping",
    timeout: float = 10.0,
    confidence_threshold: float = -1.0,
    partition_angle_offset: int = 0,
) -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Selector(
        name="Select subtree by ping",
        memory=True,
    )

    seq_sub_check = py_trees.composites.Sequence(
        name="Sequence subscribe to ping and check confidence",
        memory=True,
    )

    seq_sub_check.add_children(
        children=[
            _create_get_ping_root(ping_topic),
            _create_ping_check_root(confidence_threshold, partition_angle_offset),
        ]
    )

    retry_wait_ping = py_trees.decorators.Retry(
        name="Retry wait for good ping",
        child=seq_sub_check,
        num_failures=100_000,
    )

    timeout_wait_ping = py_trees.decorators.Timeout(
        name="Timeout wait for good ping",
        child=retry_wait_ping,
        duration=timeout,
    )

    seq_confidence_check_threshold_check_octagon_torpedo = py_trees.composites.Sequence(
        name="Sequence check and first order",
        memory=True,
    )

    seq_confidence_check_threshold_check_octagon_torpedo.add_children(
        children=[
            timeout_wait_ping,
            octagon_torpedo_execution,
        ]
    )

    root.add_children(
        children=[
            seq_confidence_check_threshold_check_octagon_torpedo,
            torpedo_octagon_execution,
        ]
    )

    return root
