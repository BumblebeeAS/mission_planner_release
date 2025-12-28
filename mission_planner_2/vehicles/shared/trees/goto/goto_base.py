#!/usr/bin/env python3


import uuid
from abc import ABC, abstractmethod
from typing import Any, Callable

import action_msgs
import action_msgs.msg as action_msgs
import py_trees
import py_trees_ros
from geometry_msgs.msg import PoseStamped

from mission_planner_2.common.config.generic_registry import SharedAction, SharedService
from mission_planner_2.common.core import shared_action_client
from mission_planner_2.vehicles.shared.trees.blackboard import convert_to_safe_name


class FromBlackboard(shared_action_client.FromBlackboard, ABC):
    """
    Base interface for all goto behaviours.
    Subclasses must minimally implement the `_gen_srv_req` and `_gen_goal` methods for the goto to work.

    Optionally if more functionality is needed overwrite the `setup` and `initialise` method, `update`
    should not need to be changed.
    """

    ACTION_GOAL_KEY = "goto_goal"

    def __init__(
        self,
        name: str,
        pose_key: str,
        action_client_type: SharedAction,
        service_client_type: SharedService,
        anchor_frame_name: str,
        generate_feedback_message: Callable[[Any], str] = None,
        wait_for_server_timeout_sec: int = -3,
        wait_for_service_timeout_sec: int = -3,
    ):
        namespace = py_trees.blackboard.Blackboard.absolute_name(
            "/", convert_to_safe_name(name) + "/" + str(uuid.uuid4()).replace("-", "")
        )

        super().__init__(
            name,
            action_client_type,
            py_trees.blackboard.Blackboard.absolute_name(
                namespace, self.ACTION_GOAL_KEY
            ),
            generate_feedback_message,
            wait_for_server_timeout_sec,
        )
        self.service_client_type = service_client_type

        self.wait_for_service_timeout_sec = wait_for_service_timeout_sec
        self.anchor_frame_name = anchor_frame_name

        # Register the pose_key on the BB as the req to be converted
        # pose_key entry should be a pose stamped
        self.blackboard.register_key(
            key="request",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                namespace="/",
                key=pose_key,
            ),
        )

        self.service_client = None

    def setup(self, **kwargs):
        """
        We ride on the super class (action_client) setup which creates the action client and the node
        using the same node instance we will create our service client
        """
        super().setup(**kwargs)

        self.service_client = self.node.service_clients[self.service_client_type.name]
        self._check_srv_setup()

    def initialise(self):
        """
        Reset internal variables and start new request

        We dont call the action clients initialise here so we have to handle resetting the vars
        """
        self.logger.debug("{}.initialise()".format(self.qualified_name))

        self._reset_internal_vars()

        # Send the service request once during initialise
        self._send_srv_request()

    def update(self):
        """
        Check whether if underlying service server has succeeded, is running,
        or has cancelled/aborted and map these to behaviour return states
        """
        self.logger.debug("{}.update()".format(self.qualified_name))

        if self.service_future is None:
            # No request on blackboard or wrong request type or unready server
            self.feedback_message = "no service request to send"
            return py_trees.common.Status.FAILURE
        elif not self.service_future.done():
            # service has been called but has yet to return a result
            return py_trees.common.Status.RUNNING

        # at this point service is done
        if not self.service_future.result().tf_success:
            return py_trees.common.Status.FAILURE

        # check that in the callback attached the new attr has been set
        # also checks if the attr has been set to True which implies that the send_goal_req has been
        # run to completion
        # this ensures that the call to send_goal_request has completed
        if not self.is_goal_sent:
            return py_trees.common.Status.RUNNING

        # no race condition cuz is RW WR either way the code wont break
        if self.send_goal_future is None:
            self.feedback_message = "no goal to send"
            return py_trees.common.Status.FAILURE
        if self.goal_handle is not None and not self.goal_handle.accepted:
            # goal was rejected
            self.feedback_message = "goal rejected"
            return py_trees.common.Status.FAILURE
        if self.result_status is None:
            return py_trees.common.Status.RUNNING
        elif not self.get_result_future.done():
            # should never get here
            self.node.get_logger().warn(
                "got result, but future not yet done [{}]".format(self.qualified_name)
            )
            return py_trees.common.Status.RUNNING
        else:
            self.node.get_logger().debug("goal result [{}]".format(self.qualified_name))
            self.node.get_logger().debug(
                "  status: {}".format(self.result_status_string)
            )
            self.node.get_logger().debug("  message: {}".format(self.result_message))
            if self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED:  # noqa
                self.feedback_message = "successfully completed"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = "failed"
                return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        """
        If running and current request has not already succeeded, cancel it.
        The behaviour transitions to new_status.
        """
        super().terminate(new_status)

        self.logger.debug(
            "{}.terminate({})".format(
                self.qualified_name,
                (
                    "{}->{}".format(self.status, new_status)
                    if self.status != new_status
                    else "{}".format(new_status)
                ),
            )
        )
        if (self.service_future is not None) and (not self.service_future.done()):
            self.service_client.remove_pending_request(self.service_future)

    def shutdown(self):
        """
        Clean up service client when shutting down
        """
        super().shutdown()

    @abstractmethod
    def _gen_srv_req(self, poses: list[PoseStamped] | PoseStamped) -> Any:
        pass

    @abstractmethod
    def _gen_goal(self, poses: list[PoseStamped]) -> Any:
        pass

    def _check_srv_setup(self):
        """
        Check if the service client is set up and ready to use.
        """
        result = None
        if self.wait_for_service_timeout_sec > 0.0:
            result = self.service_client.wait_for_service(
                timeout_sec=self.wait_for_service_timeout_sec
            )
        elif self.wait_for_service_timeout_sec == 0.0:
            result = True  # don't wait and don't check if the server is ready
        else:
            iterations = 0
            period_sec = -1.0 * self.wait_for_service_timeout_sec
            while not result:
                iterations += 1
                result = self.service_client.wait_for_service(timeout_sec=period_sec)
                if not result:
                    self.node.get_logger().warning(
                        "waiting for service server ... [{}s][{}][{}]".format(
                            iterations * period_sec,
                            self.node.resolve_service_name(
                                self.shared_action.value.topic
                            ),
                            self.qualified_name,
                        )
                    )

        if not result:
            self.feedback_message = "timed out waiting for the server [{}]".format(
                self.node.resolve_service_name(self.shared_action.value.topic)
            )
            self.node.get_logger().error(
                "{}[{}]".format(self.feedback_message, self.qualified_name)
            )
            raise py_trees_ros.exceptions.TimedOutError(self.feedback_message)
        else:
            self.feedback_message = "... connected to service server [{}]".format(
                self.node.resolve_service_name(self.shared_action.value.topic)
            )
            self.node.get_logger().info(
                "{}[{}]".format(self.feedback_message, self.qualified_name)
            )

    def _send_srv_request(self):
        """
        Send the service request to the service server.
        This method is called during initialise to send the service request and attach the callback.
        """
        poses = self.blackboard.get("request")

        try:
            if self.service_client.service_is_ready():
                self.service_future = self.service_client.call_async(
                    self._gen_srv_req(poses)
                )
                self.feedback_message = "sent service request"
                self.service_future.add_done_callback(self._srv_done_callback)
        except (KeyError, TypeError):
            pass

    def _send_goal_request(self, poses):
        """
        Send the goal request to the action server.
        This method is called after the service call has been completed and the poses have been converted.
        Called by the `_srv_done_callback` method.
        """
        goal = self._gen_goal(poses)

        # send_goal_request sets the send_goal_future attr
        self.send_goal_request(goal)
        # separate flag to check goal has been sent in that case the send_goal_future must have been set
        self.is_goal_sent = True
        self.feedback_message = "sent action goal request"

    def _srv_done_callback(self, fut):
        resp = fut.result()
        if not resp.tf_success:
            return

        self._send_goal_request(resp.output_poses)

    def _reset_internal_vars(self):
        """
        Reset internal variables to initial state.
        """
        # Temporary variable
        self.service_future = None

        # None declarations from super.initialise
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None

        self.result_message = None
        self.result_status = None
        self.result_status_string = None

        # New attr to track if goal has been sent
        self.is_goal_sent = False
