#!/usr/bin/env python3
import uuid
from typing import Callable

import action_msgs
import action_msgs.msg as action_msgs
import py_trees
from bb_uav_msgs.action import GoToPosition
from geometry_msgs.msg import PoseStamped
from mission_planner_2.common.core import shared_action_client
from mission_planner_2.vehicles.shared.trees.blackboard import convert_to_safe_name
from mission_planner_2.vehicles.uav2.config.node_registry import UAV2SharedAction
from rclpy.node import Node


class FromBlackboard(shared_action_client.FromBlackboard):
    """
    Interface to communicate with UAV2's GoToPosition action using a pose in the
    stored in the blackboard.

    Execution Flow:
    1. Read PoseStamped from blackboard at the specified key
    2. Convert the pose to the control frame using the conversion service
    3. Generate a Locomotion goal from the converted pose
    4. Send the goal to the Locomotion Action Server
    5. Monitor the action execution and return appropriate status

    Args:
        name (str): The name of the behaviour.
        parent_namespace (str): The namespace of the parent behaviour to properly scope blackboard variables.
        pose_key (str): The key of the blackboard variable to read the pose from. The BB entry at this key
                        **MUST** be of type `geometry_msgs.msg.PoseStamped`.
        x_threshold (float): The threshold for the x coordinate.
        y_threshold (float): The threshold for the y coordinate.
        z_threshold (float): The threshold for the z coordinate.
        generate_feedback_message (callable, optional): A callable to generate feedback messages.
        wait_for_server_timeout_sec (float): Timeout for waiting for the action server to be ready.
                                            Negative values will repeatedly try with the absolute value as the period.
                                            Zero will not wait at all. (default: -3).

    Returns:
        py_trees.common.Status.SUCCESS: When the locomotion action completes successfully
        py_trees.common.Status.RUNNING: While the service call or action is in progress
        py_trees.common.Status.FAILURE: If the pose is not found on the blackboard, service call fails,
                                       goal is rejected, or action fails
    """

    ACTION_GOAL_KEY = "goto_goal"

    def __init__(
        self,
        name: str,
        pose_key: str,
        x_threshold: float = 0.03,
        y_threshold: float = 0.03,
        z_threshold: float = 0.03,
        generate_feedback_message: Callable | None = None,
        wait_for_server_timeout_sec: int = -3,
    ):
        namespace = py_trees.blackboard.Blackboard.absolute_name(
            "/", convert_to_safe_name(name) + "/" + str(uuid.uuid4()).replace("-", "")
        )

        super().__init__(
            name,
            UAV2SharedAction.GOTO,
            py_trees.blackboard.Blackboard.absolute_name(
                namespace, self.ACTION_GOAL_KEY
            ),
            generate_feedback_message,
            wait_for_server_timeout_sec,
        )

        self.x_threshold = x_threshold
        self.y_threshold = y_threshold
        self.z_threshold = z_threshold

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

        self.node: Node

    def initialise(self):
        """
        Reset internal variables and start new request

        We dont call the action clients initialise here so we have to handle resetting the vars
        """
        self.logger.debug("{}.initialise()".format(self.qualified_name))

        # None declarations from super.initialise
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None

        self.result_message = None
        self.result_status = None
        self.result_status_string = None
        self.is_goal_sent = False

        pose = self.blackboard.get("request")
        self._send_goal_request(pose)

    def update(self):
        """
        Check whether if underlying service server has succeeded, is running,
        or has cancelled/aborted and map these to behaviour return states
        """
        self.logger.debug("{}.update()".format(self.qualified_name))

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

    def shutdown(self):
        """
        Clean up service client when shutting down
        """
        super().shutdown()

    def _gen_goal(self, pose: PoseStamped):
        goal_msg = GoToPosition.Goal()
        goal_msg.x = pose.pose.position.x
        goal_msg.y = pose.pose.position.y
        goal_msg.z = pose.pose.position.z
        goal_msg.x_threshold = self.x_threshold
        goal_msg.y_threshold = self.y_threshold
        goal_msg.z_threshold = self.z_threshold
        return goal_msg

    def _send_goal_request(self, pose: PoseStamped):
        """
        Send the goal request to the action server.
        This method is called after the service call has been completed and the poses have been converted.
        """
        goal = self._gen_goal(pose)

        # send_goal_request sets teh send_goal_future attr
        self.send_goal_request(goal)
        # separate flag to check goal has been sent in that case the send_goal_future must have been set
        self.is_goal_sent = True
        self.feedback_message = "sent action goal request"


class FromConstant(FromBlackboard):
    """
    Interface to communicate with controls `Locomotion Action server` using a constant pose.

    Instead of reading a pose from the blackboard, this behavior uses a pose provided at initialization.

    Args:
        name (str): The name of the behaviour.
        pose (PoseStamped): The constant pose to use for the goto action.
        x_threshold (float): The threshold for the x coordinate.
        y_threshold (float): The threshold for the y coordinate.
        z_threshold (float): The threshold for the z coordinate.
        generate_feedback_message (callable, optional): A callable to generate feedback messages.
        wait_for_server_timeout_sec (float): Timeout for waiting for the action server to be ready.
    """

    def __init__(
        self,
        name,
        pose: PoseStamped,
        x_threshold: float = 0.03,
        y_threshold: float = 0.03,
        z_threshold: float = 0.03,
        generate_feedback_message=None,
        wait_for_server_timeout_sec=-3,
    ):
        namespace = py_trees.blackboard.Blackboard.absolute_name(
            "/", convert_to_safe_name(name)
        )
        pose_key = py_trees.blackboard.Blackboard.absolute_name(
            namespace, f"pose_{str(uuid.uuid4()).replace('-', '')}"
        )

        super().__init__(
            name=name,
            pose_key=pose_key,
            x_threshold=x_threshold,
            y_threshold=y_threshold,
            z_threshold=z_threshold,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        )

        self.blackboard.register_key(
            key="request",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                namespace="/",
                key=pose_key,
            ),
        )
        self.blackboard.set(name="request", value=pose)
