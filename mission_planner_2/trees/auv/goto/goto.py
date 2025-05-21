#!/usr/bin/env python3


"""
Behaviours for ROS services
"""

from typing import Callable

import action_msgs
import action_msgs.msg as action_msgs
import py_trees
import py_trees_ros
from bb_controls_msgs.action import Locomotion
from bb_planner_msgs.srv import GetPoseToControlsFrame
from geometry_msgs.msg import PoseStamped
from numpy import rad2deg
from transforms3d.euler import quat2euler

from mission_planner_2.commons.blackboard import convert_to_safe_name


class FromBlackboard(py_trees_ros.action_clients.FromBlackboard):
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
    from mission_planner_2.trees.auv.goto import goto
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

    ACTION_TYPE = Locomotion
    ACTION_NAME = "/auv4/controls"
    ACTION_GOAL_KEY = "goto_goal"
    SERVICE_TYPE = GetPoseToControlsFrame
    SERVICE_NAME = "/auv4/convert_to_controls_pose"

    def __init__(
        self,
        name: str,
        parent_namespace: str,
        pose_key: str,
        anchor_frame_name: str = "auv4/base_link_ned",
        generate_feedback_message: Callable | None = None,
        wait_for_server_timeout_sec: int = -3,
        wait_for_service_timeout_sec: int = -3,
    ):
        self.safe_name = convert_to_safe_name(name)
        self.parent_namespace = parent_namespace
        self.namespace = py_trees.blackboard.Blackboard.absolute_name(
            parent_namespace, self.safe_name
        )

        super().__init__(
            name,
            self.ACTION_TYPE,
            self.ACTION_NAME,
            py_trees.blackboard.Blackboard.absolute_name(
                self.namespace, self.ACTION_GOAL_KEY
            ),
            generate_feedback_message,
            wait_for_server_timeout_sec,
        )

        self.wait_for_service_timeout_sec = wait_for_service_timeout_sec
        self.anchor_frame_name = anchor_frame_name

        # Register the pose_key on the BB as the req to be converted
        # pose_key entry should be a pose stamped
        self.blackboard.register_key(
            key="request",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                self.parent_namespace, key=pose_key
            ),
        )

        self.service_client = None

    def setup(self, **kwargs):
        """
        We ride on the super class (action_client) setup which creates the action client and the node
        using the same node instance we will create our service client
        """
        super().setup(**kwargs)

        self.service_client = self.node.create_client(
            srv_type=self.SERVICE_TYPE, srv_name=self.SERVICE_NAME
        )

        self._check_srv_setup()

    def initialise(self):
        """
        Reset internal variables and start new request

        We dont call the action clients initialise here so we have to handle resetting the vars
        """
        self.logger.debug("{}.initialise()".format(self.qualified_name))

        # Temporary variable
        self.service_future = None

        # FIXME: problem was they dont raise error in sending of the goal request then use the none
        # check to see if success or not in the original action client node dont use goal handle
        # as a way to check the BB var exists (fixed using is_goal_sent new attr)

        try:
            if self.service_client.service_is_ready():
                self.service_future = self.service_client.call_async(
                    self._gen_srv_req(self.blackboard.request)
                )
            self.feedback_message = "sent service request"
            self.service_future.add_done_callback(self._srv_done_callback)
        except (KeyError, TypeError):
            pass  # self.service_future resolves to None

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
        if not hasattr(self, "is_goal_sent") and not self.is_goal_sent:
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
        self.service_client.destroy()

    def _gen_srv_req(self, pose):
        request = GetPoseToControlsFrame.Request()
        print(type(pose))
        request.input_pose = pose
        request.anchor_frame_name = self.anchor_frame_name
        return request

    def _gen_goal(self, service_response: PoseStamped):
        output_pose = service_response.pose

        goal_msg = Locomotion.Goal()

        # Set the required fields
        goal_msg.move_rel = False
        goal_msg.depth_rel = False
        goal_msg.heading_rel = False

        try:
            goal_msg.depth_ctrl = Locomotion.Goal.DEPTH_MODE_DEPTH
        except Exception as e:
            print(e)
            goal_msg.depth_ctrl = 0

        goal_msg.specified_heading = True

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

        # Set the setpoints
        goal_msg.forward_setpoints = [output_pose.position.x]
        goal_msg.sidemove_setpoints = [output_pose.position.y]
        goal_msg.depth_setpoints = [output_pose.position.z]
        goal_msg.heading_setpoints = [yaw]

        # Lists that need to be populated but aren't used
        goal_msg.roll_setpoints = [0.0]
        goal_msg.pitch_setpoints = [0.0]
        goal_msg.altitude_setpoints = []

        return goal_msg

    def _check_srv_setup(self):
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
                            self.node.resolve_service_name(self.SERVICE_NAME),
                            self.qualified_name,
                        )
                    )

        if not result:
            self.feedback_message = "timed out waiting for the server [{}]".format(
                self.node.resolve_service_name(self.SERVICE_NAME)
            )
            self.node.get_logger().error(
                "{}[{}]".format(self.feedback_message, self.qualified_name)
            )
            raise py_trees_ros.exceptions.TimedOutError(self.feedback_message)
        else:
            self.feedback_message = "... connected to service server [{}]".format(
                self.node.resolve_service_name(self.SERVICE_NAME)
            )
            self.node.get_logger().info(
                "{}[{}]".format(self.feedback_message, self.qualified_name)
            )

    def _srv_done_callback(self, fut):
        resp = fut.result()
        if not resp.tf_success:
            return
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None

        self.result_message = None
        self.result_status = None
        self.result_status_string = None
        self.is_goal_sent = False

        goal = self._gen_goal(resp.output_pose)

        # send_goal_request sets teh send_goal_future attr
        self.send_goal_request(goal)
        # separate flag to check goal has been sent in that case the send_goal_future must have been set
        self.is_goal_sent = True
        self.feedback_message = "sent action goal request"


class FromConstant(FromBlackboard):
    """
    Interface to communicate with controls `Locomotion Action server` using a constant pose.

    Instead of reading a pose from the blackboard, this behavior uses a pose provided at initialization.

    Example usage:
    ```python
    from mission_planner_2.trees.auv.goto import goto_mine

    root.add_children(
        [
            goto_mine.FromConstant(
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
        parent_namespace,
        pose,
        anchor_frame_name="auv4/base_link_ned",
        generate_feedback_message=None,
        wait_for_server_timeout_sec=-3,
        wait_for_service_timeout_sec=-3,
    ):
        import uuid

        pose_key = f"pose_{str(uuid.uuid4()).replace('-', '')}"

        super().__init__(
            name=name,
            parent_namespace=parent_namespace,
            pose_key=pose_key,
            anchor_frame_name=anchor_frame_name,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
            wait_for_service_timeout_sec=wait_for_service_timeout_sec,
        )

        # Store the pose directly on the blackboard
        self.blackboard.register_key(
            key="request",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                self.parent_namespace, key=pose_key
            ),
        )
        self.blackboard.set(name="request", value=pose)
