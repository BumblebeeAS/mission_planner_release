import operator

import py_trees
import py_trees_ros
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Bool

from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

_BUTTON_RESPONSE_KEY = fk("button_response")


def create_wait_for_button_root(
    button_topic: str = "/auv4/button/left", num_retries: int = 1000000
):
    seq_read_button = py_trees.composites.Sequence(
        "Read and check button sequence",
        memory=True,
    )

    sub_button = py_trees_ros.subscribers.ToBlackboard(
        name="Read from button topic",
        topic_name=button_topic,
        topic_type=Bool,
        qos_profile=qos_profile_system_default,
        blackboard_variables={_BUTTON_RESPONSE_KEY: None},
    )

    check_button_pressed = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check if button pressed",
        check=py_trees.common.ComparisonExpression(
            variable=_BUTTON_RESPONSE_KEY,
            value=True,
            operator=lambda x, y: operator.eq(x.data, y),
        ),
    )

    seq_read_button.add_children([sub_button, check_button_pressed])

    root = py_trees.decorators.Retry(
        name="Retry button until success",
        child=seq_read_button,
        num_failures=num_retries,
    )

    return root
