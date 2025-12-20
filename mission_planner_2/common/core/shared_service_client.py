import typing
import uuid

import py_trees
import py_trees_ros
from mission_planner_2.common.config.generic_registry import SharedService
from py_trees_ros import exceptions


class FromBlackboard(py_trees_ros.service_clients.FromBlackboard):
    def __init__(
        self,
        name: str,
        shared_service: SharedService,
        key_request: str,
        key_response: str = None,
        wait_for_server_timeout_sec: float = -3,
    ):
        self.shared_service = shared_service
        super().__init__(
            name,
            self.shared_service.value.type,
            self.shared_service.value.topic,
            key_request,
            key_response,
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

        self.service_client = self.node.service_clients[self.shared_service.name]
        result = None
        if self.wait_for_server_timeout_sec > 0.0:
            result = self.service_client.wait_for_service(
                timeout_sec=self.wait_for_server_timeout_sec
            )
        elif self.wait_for_server_timeout_sec == 0.0:
            result = True  # don't wait and don't check if the server is ready
        else:
            iterations = 0
            period_sec = -1.0 * self.wait_for_server_timeout_sec
            while not result:
                iterations += 1
                result = self.service_client.wait_for_service(timeout_sec=period_sec)
                if not result:
                    self.node.get_logger().warning(
                        "waiting for service server ... [{}s][{}][{}]".format(
                            iterations * period_sec,
                            self.service_name,
                            self.qualified_name,
                        )
                    )
        if not result:
            self.feedback_message = "timed out waiting for the server [{}]".format(
                self.service_name
            )
            self.node.get_logger().error(
                "{}[{}]".format(self.feedback_message, self.qualified_name)
            )
            raise exceptions.TimedOutError(self.feedback_message)
        else:
            self.feedback_message = "... connected to service server [{}]".format(
                self.service_name
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
        shared_service: SharedService,
        service_request: typing.Any,
        key_response: str = None,
        wait_for_server_timeout_sec: float = -3,
    ):
        unique_id = uuid.uuid4()
        key_request = "/request_" + str(unique_id)
        super().__init__(
            name=name,
            shared_service=shared_service,
            key_request=key_request,
            key_response=key_response,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        )
        # parent already instantiated a blackboard client
        self.blackboard.register_key(
            key=key_request,
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.set(name=key_request, value=service_request)
