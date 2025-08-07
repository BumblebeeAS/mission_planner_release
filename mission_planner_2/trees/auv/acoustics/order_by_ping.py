import py_trees
import py_trees_ros
from rclpy.qos import qos_profile_system_default

from bb_sensor_msgs.msg import Ping

from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)


NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

_PING_RESPONSE_KEY = fk("ping")


def create_order_by_ping_root(
        first_subtree_func,
        second_subtree_func,
        ping_topic: str = "/sensors/ping",
        timeout: float = 20.0,
        confidence_threshold: float = 0.8,
) -> py_trees.behaviour.Behaviour:
    
    sel_subtree = py_trees.composites.Selector(
        name="Select subtree by ping",
        memory=True
    )

    seq_sub_check = py_trees.composites.Sequence(
        name="Sequence subscribe to ping and check confidence",
        memory=True
    )

    sub_ping = py_trees_ros.subscribers.ToBlackboard(
        name="Subcribe to ping",
        topic_name=ping_topic,
        topic_type=Ping,
        qos_profile=qos_profile_system_default,
        blackboard_variables={_PING_RESPONSE_KEY: None}
    )

    check_ping_confidence = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check ping confidence",
        check=py_trees.common.ComparisonExpression(
            variable=_PING_RESPONSE_KEY,
            value=confidence_threshold,
            operator=lambda x, y: x.confidence >= y
        )
    )

    check_ping_direction = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check ping direction",
        check=py_trees.common.ComparisonExpression(
            variable=_PING_RESPONSE_KEY,
            value=180,
            operator=lambda x, y: x.doa_deg < y
        )
    )

    seq_sub_check.add_children(
        children=[
            sub_ping,
            check_ping_confidence
        ]
    )

    retry_wait_ping = py_trees.decorators.Retry(
        name="Retry wait for good ping",
        child=seq_sub_check,
        num_failures=100_000
    )

    timeout_wait_ping = py_trees.decorators.Timeout(
        name="Timeout wait for good ping",
        child=retry_wait_ping,
        duration=timeout
    )

    seq_confidence_check_threshold_check_first_order = py_trees.composites.Sequence(
        name="Sequence check and first order",
        memory=True
    )

    seq_first_order = py_trees.composites.Sequence(
        name="Sequence first order",
        memory=True
    )

    seq_second_order = py_trees.composites.Sequence(
        name="Sequence second order",
        memory=True
    )

    seq_first_order.add_children(
        children=[
            first_subtree_func(),
            second_subtree_func()
        ]
    )

    seq_second_order.add_children(
        children=[
            second_subtree_func(),
            first_subtree_func()
        ]
    )

    seq_confidence_check_threshold_check_first_order.add_children(
        children=[
            timeout_wait_ping,
            check_ping_direction,
            seq_first_order
        ]
    )

    sel_subtree.add_children(
        children=[
            seq_confidence_check_threshold_check_first_order,
            seq_second_order
        ]
    )

    return sel_subtree
