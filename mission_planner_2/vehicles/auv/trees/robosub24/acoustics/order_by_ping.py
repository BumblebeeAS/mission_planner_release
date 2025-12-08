import py_trees
import py_trees_ros
from bb_auv_msgs.action import Grabber
from bb_perception_msgs.srv import GetPingCount

from mission_planner_2.common.core import shared_action_client
from mission_planner_2.common.util.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.vehicles.auv.config.node_registry import SharedAction

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

NUM_PINGS_REQUIRED = 3
PINGER_TIMER = 10.0
_PING_RESPONSE_KEY = fk("ping")


def _create_grabber_root(is_open: bool = False):
    grabber = shared_action_client.FromConstant(
        name="Close grabber for pings",
        shared_action=SharedAction.GRABBER,
        action_goal=Grabber.Goal(
            command=65535 if is_open else 5,
            tolerance=0,
            timeout_ms=5000,
        ),
    )

    force_succeed_grabber = py_trees.decorators.FailureIsSuccess(
        name="force succed close grabber",
        child=grabber,
    )

    return force_succeed_grabber


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
            check_ping_direction,
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
    root = py_trees.composites.Sequence(
        name="Order by ping seq",
        memory=True,
    )

    sel_task = py_trees.composites.Selector(
        name="Select subtree by ping",
        memory=True,
    )

    seq_sub_check = py_trees.composites.Sequence(
        name="Sequence subscribe to ping and check confidence",
        memory=True,
    )

    req = GetPingCount.Request()
    req.enable = True
    req.num_pings_required = NUM_PINGS_REQUIRED
    req.partition_angle = float(partition_angle_offset)

    srv_enable_acoustic = py_trees_ros.service_clients.FromConstant(
        name="Enable clustering",
        service_type=GetPingCount,
        service_name="/auv4/sensors/get_ping_count",
        service_request=req,
    )

    timer = py_trees.timers.Timer(
        name="timer",
        duration=PINGER_TIMER,
    )

    req_disable = GetPingCount.Request()
    req_disable.enable = False
    req_disable.num_pings_required = NUM_PINGS_REQUIRED
    req_disable.partition_angle = float(partition_angle_offset)

    srv_disable_acoustic = py_trees_ros.service_clients.FromConstant(
        name="Disable clustering",
        service_type=GetPingCount,
        service_name="/auv4/sensors/get_ping_count",
        service_request=req_disable,
        key_response=_PING_RESPONSE_KEY,
    )

    default_response = GetPingCount.Response()
    default_response.is_left = False

    set_ping_response_key_default = py_trees.behaviours.SetBlackboardVariable(
        name="Set ping response key default",
        variable_name=_PING_RESPONSE_KEY,
        variable_value=default_response,
        overwrite=True,
    )

    seq_sub_check.add_children(
        children=[
            set_ping_response_key_default,
            srv_enable_acoustic,
            timer,
            srv_disable_acoustic,
            py_trees.behaviours.CheckBlackboardVariableValue(
                name="check is not left",
                check=py_trees.common.ComparisonExpression(
                    variable=_PING_RESPONSE_KEY,
                    value=False,
                    operator=lambda x, y: x.is_left == y,
                ),
            ),
        ]
    )

    seq_confidence_check_threshold_check_octagon_torpedo = py_trees.composites.Sequence(
        name="Sequence check and first order",
        memory=True,
    )

    seq_fail = py_trees.composites.Sequence(
        name="Sequence fail",
        memory=True,
    )

    seq_fail.add_children(
        children=[
            _create_grabber_root(is_open=True),
            torpedo_octagon_execution,
        ]
    )

    seq_confidence_check_threshold_check_octagon_torpedo.add_children(
        children=[
            seq_sub_check,
            _create_grabber_root(is_open=True),
            octagon_torpedo_execution,
        ]
    )

    sel_task.add_children(
        children=[
            seq_confidence_check_threshold_check_octagon_torpedo,
            seq_fail,
        ]
    )

    root.add_children(
        [
            _create_grabber_root(is_open=False),
            sel_task,
        ]
    )

    return root
