#!/usr/bin/env python3


import time
import uuid
from typing import Any, Callable

import action_msgs
import action_msgs.msg as action_msgs
import py_trees
from bb_controls_msgs.action import Locomotion
from bb_planner_msgs.srv import GetPoseToControlsFrame
from geometry_msgs.msg import PoseStamped
from numpy import rad2deg
from rclpy.task import Future
from transforms3d.euler import quat2euler

from mission_planner_2.vehicles.auv.config.node_registry import (
    AUVSharedAction,
    AUVSharedService,
)
from mission_planner_2.vehicles.shared.trees.blackboard import convert_to_safe_name
from mission_planner_2.vehicles.shared.trees.goto import goto_base


class FromBlackboard(goto_base.FromBlackboard):
    """
    Interface to communicate with controls `Locomotion Action Server` using a pose stored in the blackboard.

    This behavior reads a PoseStamped message from the blackboard, converts it to the appropriate
    control frame using a ROS service, and then sends it as a goal to the Locomotion Action Server.
    The behavior will return SUCCESS when the locomotion action completes successfully, or FAILURE
    if any step in the process fails.

    Execution Flow:
    1. Read PoseStamped from blackboard at the specified key
    2. Convert the pose to the control frame using the conversion service
    3. Generate a Locomotion goal from the converted pose
    4. Send the goal to the Locomotion Action Server
    5. Monitor the action execution and return appropriate status

    Example usage:
    ```python
    from mission_planner_2.vehicles.auv.trees.robosub24.goto import goto
    from mission_planner_2.commons.blackboard import create_stamped_pose

    NAMESPACE = "/auv4/mission_1"

    pose_key = "input_pose_to_goto"
    full_key = py_trees.blackboard.Blackboard.absolute_name(NAMESPACE, key=pose_key)

    root.add_children(
        [
            py_trees.behaviours.SetBlackboardVariable(
                name="set_goto_1",
                variable_name=full_key,
                variable_value=create_stamped_pose("your_frame"),
                overwrite=True,
            ),
            goto.FromBlackboard(
                name="goto",
                parent_namespace=NAMESPACE,
                pose_key=pose_key,
            ),
            goto.FromConstant(
                name="goto_2",
                parent_namespace=NAMESPACE,
                pose=create_stamped_pose("your_frame"),
            ),
        ]
    )
    ```

    Args:
        name (str): The name of the behaviour.
        parent_namespace (str): The namespace of the parent behaviour to properly scope blackboard variables.
        pose_key (str): The key of the blackboard variable to read the pose from. The BB entry at this key
                        **MUST** be of type `geometry_msgs.msg.PoseStamped`.
        anchor_frame_name (str): The name of the frame that is to be brought to the target pose
                                (default: "auv4/base_link_ned").
        generate_feedback_message (callable, optional): A callable to generate feedback messages.
        wait_for_server_timeout_sec (float): Timeout for waiting for the action server to be ready.
                                            Negative values will repeatedly try with the absolute value as the period.
                                            Zero will not wait at all. (default: -3).
        wait_for_service_timeout_sec (float): Timeout for waiting for the service to be ready.
                                             Same timeout policy as wait_for_server_timeout_sec (default: -3).

    Returns:
        py_trees.common.Status.SUCCESS: When the locomotion action completes successfully
        py_trees.common.Status.RUNNING: While the service call or action is in progress
        py_trees.common.Status.FAILURE: If the pose is not found on the blackboard, service call fails,
                                       goal is rejected, or action fails
    """

    def __init__(
        self,
        name: str,
        pose_key: str,
        anchor_frame_name: str = "auv4/base_link_ned",
        specified_heading: bool = True,
        ignore_depth: bool = False,
        x_threshold: float = 0.03,
        y_threshold: float = 0.03,
        z_threshold: float = 0.03,
        yaw_threshold: float = 0.01,
        stabilize_duration: int = 5,
        generate_feedback_message: Callable[[Any], str] = None,
        wait_for_server_timeout_sec: int = -3,
        wait_for_service_timeout_sec: int = -3,
        is_relative_movement: bool = False,
        depth_override_value: float | None = None,
    ):

        super().__init__(
            name,
            pose_key,
            action_client_type=AUVSharedAction.LOCOMOTION,
            service_client_type=AUVSharedService.CONVERT_TO_CONTROLS_POSE,
            anchor_frame_name=anchor_frame_name,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            wait_for_service_timeout_sec=wait_for_service_timeout_sec,
        )

        # Set AUV specific parameters
        self.specified_heading = specified_heading
        self.ignore_depth = ignore_depth
        self.x_threshold = x_threshold
        self.y_threshold = y_threshold
        self.z_threshold = z_threshold
        self.yaw_threshold = yaw_threshold
        self.stabilize_duration = stabilize_duration
        self.is_relative_movement = is_relative_movement
        self.depth_override_value = depth_override_value

    def setup(self, **kwargs):
        """
        Call the grandparent setup method.
        """
        super(goto_base.FromBlackboard, self).setup(**kwargs)

        if self.is_relative_movement:
            return

        self.service_client = self.node.service_clients[self.service_client_type.name]
        self._check_srv_setup()

    def initialise(self):
        """
        Reset internal variables and start new request

        We dont call the action clients initialise here so we have to handle resetting the vars
        """
        self.logger.debug("{}.initialise()".format(self.qualified_name))

        self._reset_internal_vars()

        poses = self.blackboard.get("request")
        # self.node.get_logger().info(f"poses: {poses}")

        if self.is_relative_movement:
            self._initialise_relative(poses)
        else:
            self._initialise_absolute(poses)

    def _initialise_relative(self, poses):
        self.service_future = Future()
        result = GetPoseToControlsFrame.Response()
        result.tf_success = True
        result.output_poses = poses
        # we set as done for the update method
        self.service_future.set_result(result)
        self._send_goal_request(poses)

    def _initialise_absolute(self, poses):
        # this used to be the old intialise method that always calls the convert pose service
        try:
            if self.service_client.service_is_ready():
                self.service_future = self.service_client.call_async(
                    self._gen_srv_req(poses)
                )
                self.feedback_message = "sent service request"
                self.service_future.add_done_callback(self._srv_done_callback)
        except (KeyError, TypeError):
            pass

    def _gen_srv_req(self, poses: list[PoseStamped] | PoseStamped):
        request = GetPoseToControlsFrame.Request()
        request.input_poses = poses if isinstance(poses, list) else [poses]
        request.anchor_frame_name = self.anchor_frame_name
        return request

    def _gen_goal(self, poses: list[PoseStamped] | PoseStamped):
        if not isinstance(poses, list):
            poses = [poses]

        output_poses = [p.pose for p in poses]

        goal_msg = Locomotion.Goal()

        if self.ignore_depth and self.depth_override_value is not None:
            raise ValueError("ignore_depth is True and depth_override_value provided")

        # Set the required fields
        goal_msg.move_rel = self.is_relative_movement
        goal_msg.depth_rel = self.ignore_depth
        goal_msg.heading_rel = self.is_relative_movement

        try:
            goal_msg.depth_ctrl = Locomotion.Goal.DEPTH_MODE_DEPTH
        except Exception as e:
            print(e)
            goal_msg.depth_ctrl = 0

        goal_msg.specified_heading = self.specified_heading

        forward_setpoints = []
        sidemove_setpoints = []
        depth_setpoints = []
        heading_setpoints = []

        for output_pose in output_poses:
            _, _, yaw = rad2deg(
                quat2euler(
                    [
                        output_pose.orientation.w,
                        output_pose.orientation.x,
                        output_pose.orientation.y,
                        output_pose.orientation.z,
                    ]
                )
            )

            forward_setpoints.append(output_pose.position.x)
            sidemove_setpoints.append(output_pose.position.y)
            if self.depth_override_value is not None:
                depth_setpoints.append(self.depth_override_value)
            elif self.ignore_depth:
                depth_setpoints.append(0.0)
            else:
                depth_setpoints.append(output_pose.position.z)
            heading_setpoints.append(yaw)

        # Set the setpoints
        goal_msg.forward_setpoints = forward_setpoints
        goal_msg.sidemove_setpoints = sidemove_setpoints
        goal_msg.depth_setpoints = depth_setpoints
        goal_msg.heading_setpoints = heading_setpoints

        # Lists that need to be populated but aren't used
        goal_msg.roll_setpoints = [0.0 for i in range(len(output_poses))]
        goal_msg.pitch_setpoints = [0.0 for i in range(len(output_poses))]
        goal_msg.altitude_setpoints = []

        goal_msg.forward_tolerance = self.x_threshold
        goal_msg.sidemove_tolerance = self.y_threshold
        goal_msg.depth_tolerance = self.z_threshold
        goal_msg.heading_tolerance = self.yaw_threshold
        goal_msg.max_correction_time = self.stabilize_duration

        return goal_msg


class FromConstant(FromBlackboard):
    """
    Interface to communicate with controls `Locomotion Action server` using a constant pose.

    Instead of reading a pose from the blackboard, this behavior uses a pose provided at initialization.

    Example usage:
    ```python
    from mission_planner_2.vehicles.auv.trees.robosub24.goto import goto

    root.add_children(
        [
            goto.FromConstant(
                name="goto_point",
                parent_namespace="/auv4/mission_1",
                pose=_create_pose(0, 0, 0)
            ),
        ]
    )
    ```

    Args:
        name (str): The name of the behaviour.
        parent_namespace (str): The namespace of the parent behaviour
        pose (PoseStamped): The constant pose to use for the goto action.
        key_action (str): The key of the blackboard variable for the action goal.
        generate_feedback_message (callable, optional): A callable to generate feedback messages.
        wait_for_server_timeout_sec (float): Timeout for waiting for the action server to be ready.
        wait_for_service_timeout_sec (float): Timeout for waiting for the service to be ready.
        anchor_frame_name (str): The name of the frame that is to be brought to the target.
    """

    def __init__(
        self,
        name,
        # parent_namespace,
        pose: PoseStamped | list[PoseStamped],
        anchor_frame_name="auv4/base_link_ned",
        specified_heading: bool = True,
        ignore_depth: bool = False,
        x_threshold: float = 0.03,
        y_threshold: float = 0.03,
        z_threshold: float = 0.03,
        yaw_threshold: float = 0.01,
        stabilize_duration: int = 5,
        generate_feedback_message: Callable[[Any], str] = None,
        wait_for_server_timeout_sec=-3,
        wait_for_service_timeout_sec=-3,
        is_relative_movement: bool = False,
        depth_override_value: float | None = None,
    ):
        if not isinstance(pose, list):
            pose = [pose]

        namespace = py_trees.blackboard.Blackboard.absolute_name(
            "/", convert_to_safe_name(name)
        )
        pose_key = py_trees.blackboard.Blackboard.absolute_name(
            namespace, f"pose_{str(uuid.uuid4()).replace('-', '')}"
        )

        super().__init__(
            name=name,
            pose_key=pose_key,
            anchor_frame_name=anchor_frame_name,
            specified_heading=specified_heading,
            ignore_depth=ignore_depth,
            x_threshold=x_threshold,
            y_threshold=y_threshold,
            z_threshold=z_threshold,
            yaw_threshold=yaw_threshold,
            stabilize_duration=stabilize_duration,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            wait_for_service_timeout_sec=wait_for_service_timeout_sec,
            is_relative_movement=is_relative_movement,
            depth_override_value=depth_override_value,
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


class NFromBlackboard(FromBlackboard):
    def __init__(
        self,
        name: str,
        pose_key: str,
        anchor_frame_name="auv4/base_link_ned",
        specified_heading: bool = True,
        ignore_depth: bool = False,
        x_threshold: float = 0.03,
        y_threshold: float = 0.03,
        z_threshold: float = 0.03,
        yaw_threshold: float = 0.01,
        stabilize_duration: int = 5,
        generate_feedback_message=None,
        wait_for_server_timeout_sec=-3,
        wait_for_service_timeout_sec=-3,
        wait_between_moves_sec=10.0,
        is_relative_movement: bool = False,
        depth_override_value: float | None = None,
    ):
        super().__init__(
            name,
            pose_key=pose_key,
            anchor_frame_name=anchor_frame_name,
            specified_heading=specified_heading,
            ignore_depth=ignore_depth,
            x_threshold=x_threshold,
            y_threshold=y_threshold,
            z_threshold=z_threshold,
            yaw_threshold=yaw_threshold,
            stabilize_duration=stabilize_duration,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            wait_for_service_timeout_sec=wait_for_service_timeout_sec,
            is_relative_movement=is_relative_movement,
            depth_override_value=depth_override_value,
        )

        self.wait_between_moves_sec = wait_between_moves_sec
        self._waiting = False
        self._wait_start_time = None

    # setup remains unchanged as it is just linking to the pose conversion service

    def _start_goal(self):
        self.logger.debug(
            "{}.initialise() or sending {} goal".format(
                self.qualified_name, self.current_idx
            )
        )

        # Temporary variable
        self.service_future = None

        # None declarations from super.initialise
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None

        self.result_message = None
        self.result_status = None
        self.result_status_string = None

        if self.is_relative_movement:
            self._initialise_relative([self.poses_list[self.current_idx]])
        else:
            self._initialise_absolute(self.poses_list[self.current_idx])

    # change initialise abit because the service request is one pose
    def initialise(self):
        """
        Reset internal variables and start new request

        We dont call the action clients initialise here so we have to handle resetting the vars
        """
        # Read the list of poses to go through
        self.poses_list = self.blackboard.get("request")
        if not isinstance(self.poses_list, list):
            self.poses_list = [self.poses_list]
        self.num_poses = len(self.poses_list)
        self.current_idx = 0
        self._start_goal()

    # in update, we check shit, if not all poses complete, start a new goal
    def update(self):
        """
        Check whether if underlying service server has succeeded, is running,
        or has cancelled/aborted and map these to behaviour return states
        """
        self.logger.debug("{}.update()".format(self.qualified_name))

        # New waiting state before moves
        if self._waiting:
            elapsed = time.monotonic() - self._wait_start_time
            if elapsed < self.wait_between_moves_sec:
                self.feedback_message = (
                    f"waiting {self.wait_between_moves_sec:.2f} before the next move"
                )
            else:
                self._waiting = False
                self._wait_start_time = None
                self._start_goal()
            return py_trees.common.Status.RUNNING

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
            if (
                self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED
                and self.current_idx == self.num_poses - 1
            ):
                self.feedback_message = f"all {self.num_poses} moves success"
                return py_trees.common.Status.SUCCESS
            elif (
                self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED
                and self.current_idx < self.num_poses
            ):  # noqa
                self.feedback_message = (
                    f"successfully completed move to pose {self.current_idx}"
                )
                self.current_idx += 1

                # Start waiting before next move
                if self.wait_between_moves_sec > 0.0:
                    self._waiting = True
                    self._wait_start_time = time.monotonic()
                else:
                    self._start_goal()
                return py_trees.common.Status.RUNNING
            else:
                self.feedback_message = "failed"
                return py_trees.common.Status.FAILURE


class NFromConstant(NFromBlackboard):
    """
    Interface to communicate with controls `Locomotion Action server` using a list of constant poses.

    Instead of reading a pose from the blackboard, this behavior uses a list of poses provided at initialization.

    Example usage:
    ```python
    from mission_planner_2.vehicles.auv.trees.robosub24.goto import goto

    root.add_children(
        [
            goto.NFromConstant(
                name="goto_points",
                parent_namespace="/auv4/mission_1",
                poses=[_create_pose(0, 0, 0), _create_pose(1, 1, 1)]
            ),
        ]
    )
    ```

    Args:
        name (str): The name of the behaviour.
        parent_namespace (str): The namespace of the parent behaviour
        poses (list[PoseStamped]): The list of constant poses to use for the goto action.
        key_action (str): The key of the blackboard variable for the action goal.
        generate_feedback_message (callable, optional): A callable to generate feedback messages.
        wait_for_server_timeout_sec (float): Timeout for waiting for the action server to be ready.
        wait_for_service_timeout_sec (float): Timeout for waiting for the service to be ready.
        anchor_frame_name (str): The name of the frame that is to be brought to the target.
    """

    def __init__(
        self,
        name,
        poses: list[PoseStamped],
        anchor_frame_name="auv4/base_link_ned",
        specified_heading: bool = True,
        ignore_depth: bool = False,
        x_threshold: float = 0.03,
        y_threshold: float = 0.03,
        z_threshold: float = 0.03,
        yaw_threshold: float = 0.01,
        stabilize_duration: int = 5,
        generate_feedback_message=None,
        wait_for_server_timeout_sec=-3,
        wait_for_service_timeout_sec=-3,
        wait_between_moves_sec=10.0,
        is_relative_movement: bool = False,
        depth_override_value: float | None = None,
    ):
        if not isinstance(poses, list):
            poses = [poses]

        namespace = py_trees.blackboard.Blackboard.absolute_name(
            "/", convert_to_safe_name(name)
        )
        pose_key = py_trees.blackboard.Blackboard.absolute_name(
            namespace, f"pose_{str(uuid.uuid4()).replace('-', '')}"
        )

        super().__init__(
            name=name,
            pose_key=pose_key,
            anchor_frame_name=anchor_frame_name,
            specified_heading=specified_heading,
            ignore_depth=ignore_depth,
            x_threshold=x_threshold,
            y_threshold=y_threshold,
            z_threshold=z_threshold,
            yaw_threshold=yaw_threshold,
            stabilize_duration=stabilize_duration,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            wait_for_service_timeout_sec=wait_for_service_timeout_sec,
            wait_between_moves_sec=wait_between_moves_sec,
            is_relative_movement=is_relative_movement,
            depth_override_value=depth_override_value,
        )

        self.blackboard.register_key(
            key="request",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                namespace="/",
                key=pose_key,
            ),
        )
        self.blackboard.set(name="request", value=poses)
