"""
Extra utility service node for mission_planner_2.
"""

import uuid
from typing import Any, Callable

import py_trees

from py_trees_ros import service_clients


class FromBlackboard(service_clients.FromBlackboard):
    """Service client node that calls a service and checks the response."""

    def __init__(
        self,
        name: str,
        service_type: Any,
        service_name: str,
        key_request: str,
        key_response: str | None = None,
        wait_for_server_timeout_sec: float = -3.0,
        check_func: Callable[[Any], bool] = lambda x: True,
    ):
        super().__init__(
            name=name,
            service_type=service_type,
            service_name=service_name,
            key_request=key_request,
            key_response=key_response,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        )
        self.check_func = check_func

    def update(self) -> py_trees.common.Status:
        """
        Update the service client node.

        Returns:
            py_trees.common.Status: The status of the service call.
        """
        status = super().update()
        if (
            status == py_trees.common.Status.FAILURE
            or status == py_trees.common.Status.RUNNING
        ):
            return status

        # self.response is set by the parent class inside update
        if self.response is None or not self.check_func(self.response):
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS


class FromConstant(FromBlackboard):
    """Service client node that calls a service and checks the response."""

    def __init__(
        self,
        name: str,
        service_type: Any,
        service_name: str,
        service_request: Any,
        key_response: str | None = None,
        wait_for_server_timeout_sec: float = -3.0,
        check_func: Callable[[Any], bool] = lambda x: True,
    ):
        unique_id = uuid.uuid4()
        key_request = f"/goal_{unique_id}"
        super().__init__(
            name=name,
            service_type=service_type,
            service_name=service_name,
            key_request=key_request,
            key_response=key_response,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            check_func=check_func,
        )
        self.blackboard.register_key(
            key=key_request,
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.set(name=key_request, value=service_request)
