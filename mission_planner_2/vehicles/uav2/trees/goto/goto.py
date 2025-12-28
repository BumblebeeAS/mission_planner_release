#!/usr/bin/env python3
import uuid
from typing import Any, Callable

import action_msgs.msg as action_msgs
import py_trees
from bb_planner_msgs.srv import GetPoseToControlsFrame
from bb_uav_msgs.action import GoToPosition
from geometry_msgs.msg import PoseStamped

from mission_planner_2.vehicles.shared.trees.blackboard import convert_to_safe_name
from mission_planner_2.vehicles.shared.trees.goto import goto_base
from mission_planner_2.vehicles.uav2.config.node_registry import (
    UAV2SharedAction,
    UAV2SharedService,
)


class FromBlackboard(goto_base.FromBlackboard):
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
        anchor_frame_name: str = "uav2/base_link_frd",
        x_threshold: float = 0.03,
        y_threshold: float = 0.03,
        z_threshold: float = 0.03,
        generate_feedback_message: Callable[[Any], str] = None,
        wait_for_server_timeout_sec: int = -3,
        wait_for_service_timeout_sec: int = -3,
    ):
        super().__init__(
            name,
            pose_key,
            action_client_type=UAV2SharedAction.GOTO,
            service_client_type=UAV2SharedService.CONVERT_TO_CONTROLS_POSE,
            anchor_frame_name=anchor_frame_name,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            wait_for_service_timeout_sec=wait_for_service_timeout_sec,
        )

        self.x_threshold = x_threshold
        self.y_threshold = y_threshold
        self.z_threshold = z_threshold

    def _gen_goal(self, poses: list[PoseStamped]):
        goal_msg = GoToPosition.Goal()
        pose = poses[0]
        goal_msg.x = pose.pose.position.x
        goal_msg.y = pose.pose.position.y
        goal_msg.z = pose.pose.position.z
        goal_msg.x_threshold = self.x_threshold
        goal_msg.y_threshold = self.y_threshold
        goal_msg.z_threshold = self.z_threshold
        return goal_msg

    def _gen_srv_req(self, poses: list[PoseStamped] | PoseStamped):
        req = GetPoseToControlsFrame.Request()
        if isinstance(poses, list):
            raise ValueError(
                "UAV2 GoToPosition action only supports single PoseStamped goals."
            )
        req.input_poses = [poses]
        req.anchor_frame_name = self.anchor_frame_name
        return req


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
        generate_feedback_message: Callable[[Any], str] = None,
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
