import typing
import uuid

import py_trees
import py_trees_ros
from py_trees_ros import exceptions

from mission_planner_2.common.config.generic_registry import SharedAction


class FromBlackboard(py_trees_ros.action_clients.FromBlackboard):
    def __init__(
        self,
        name: str,
        shared_action: SharedAction,
        key: str,
        generate_feedback_message: typing.Callable[[typing.Any], str] = None,
        wait_for_server_timeout_sec: float = -3,
    ):
        self.shared_action = shared_action
        super().__init__(
            name,
            self.shared_action.value.type,
            self.shared_action.value.topic,
            key,
            generate_feedback_message,
            wait_for_server_timeout_sec,
        )

    def setup(self, **kwargs):
        self.logger.debug("{}.setup()".format(self.qualified_name))
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.qualified_name
            )
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.action_client = self.node.action_clients[self.shared_action.name]
        result = None
        if self.wait_for_server_timeout_sec > 0.0:
            result = self.action_client.wait_for_server(
                timeout_sec=self.wait_for_server_timeout_sec
            )
        elif self.wait_for_server_timeout_sec == 0.0:
            result = True  # don't wait and don't check if the server is ready
        else:
            iterations = 0
            period_sec = -1.0 * self.wait_for_server_timeout_sec
            while not result:
                iterations += 1
                result = self.action_client.wait_for_server(timeout_sec=period_sec)
                if not result:
                    self.node.get_logger().warning(
                        "waiting for action server ... [{}s][{}][{}]".format(
                            iterations * period_sec,
                            self.action_name,
                            self.qualified_name,
                        )
                    )
        if not result:
            self.feedback_message = "timed out waiting for the server [{}]".format(
                self.action_name
            )
            self.node.get_logger().error(
                "{}[{}]".format(self.feedback_message, self.qualified_name)
            )
            raise exceptions.TimedOutError(self.feedback_message)
        else:
            self.feedback_message = "... connected to action server [{}]".format(
                self.action_name
            )
            self.node.get_logger().info(
                "{}[{}]".format(self.feedback_message, self.qualified_name)
            )

    def shutdown(self):
        return


class FromConstant(FromBlackboard):
    def __init__(
        self,
        name: str,
        shared_action: SharedAction,
        action_goal: typing.Any,
        generate_feedback_message: typing.Callable[[typing.Any], str] = None,
        wait_for_server_timeout_sec: float = -3,
    ):
        unique_id = uuid.uuid4()
        key = "/goal_" + str(unique_id)
        super().__init__(
            name=name,
            shared_action=shared_action,
            key=key,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        )
        # parent already instantiated a blackboard client
        self.blackboard.register_key(
            key=key,
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.set(name=key, value=action_goal)
