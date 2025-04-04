#!/usr/bin/env python3


"""
Behaviours for ROS services
"""

import typing
import uuid

import action_msgs
import action_msgs.msg as action_msgs
import py_trees
import py_trees_ros
from bb_controls_msgs.action import Locomotion
from bb_planner_msgs.srv import GetPoseToControlsFrame
from geometry_msgs.msg import PoseStamped
from numpy import rad2deg
from transforms3d.euler import quat2euler


class FromBlackboard(py_trees_ros.action_clients.FromBlackboard):
    """
    Interface to communicate with controls `Locomotion Action server`

    Example usage:
    ```python
    from mission_planner_2.trees.auv.goto import goto_mine

    root.add_children(
        [
            py_trees.behaviours.SetBlackboardVariable(
                name="set_goto_1",
                variable_name="input_pose_to_goto",
                variable_value=_create_pose(0, 0, 0),
                overwrite=True,
            ),
            goto_mine.FromBlackboard(name="goto", pose_key="input_pose_to_goto"),
        ]
    )
    ```

    Args:
        name (str): The name of the behaviour.
        pose_key (str): The key of the blackboard variable to read from, the BB entry at this key **MUST**
                        be of type `PoseStamped`.
        action_type (type): The type of the controls action server.
        action_name (str): The name of the controls action server.
        service_type (type): The type of the service used to convert poses.
        service_name (str): The name of the service used to convert poses.
        key_action (str): The key of the blackboard variable for the action goal.
        generate_feedback_message (callable, optional): A callable to generate feedback messages.
        wait_for_server_timeout_sec (float): Timeout for waiting for the action server to be ready.
        wait_for_service_timeout_sec (float): Timeout for waiting for the service to be ready.
        move_rel (bool): Whether the movement is relative.
        depth_rel (bool): Whether the depth is relative.
        heading_rel (bool): Whether the heading is relative.
        specified_heading (bool): Whether a specific heading is required.
        roll_setpoints (list[float]): List of roll setpoints.
        pitch_setpoints (list[float]): List of pitch setpoints.
        altitude_setpoints (list[float]): List of altitude setpoints.
    """

    def __init__(
        self,
        name,
        pose_key,
        action_type=Locomotion,
        action_name="/auv4/controls",
        service_type=GetPoseToControlsFrame,
        service_name="/auv4/convert_to_controls_pose",
        key_action="placeholder",
        generate_feedback_message=None,
        wait_for_server_timeout_sec=-3,
        wait_for_service_timeout_sec=-3,
        move_rel=False,
        depth_rel=False,
        heading_rel=False,
        specified_heading=True,
        roll_setpoints=[0.0],
        pitch_setpoints=[0.0],
        altitude_setpoints=[],
    ):
        super().__init__(
            name,
            action_type,
            action_name,
            key_action,
            generate_feedback_message,
            wait_for_server_timeout_sec,
        )
        self.service_type = service_type
        self.service_name = service_name
        self.wait_for_service_timeout_sec = wait_for_service_timeout_sec

        # Register the pose_key on the BB as the req to be converted
        # pose_key entry should be a pose stamped
        self.blackboard.register_key(
            key="request",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key=pose_key),
        )

        self.service_client = None

        self.move_rel = move_rel
        self.depth_rel = depth_rel
        self.heading_rel = heading_rel
        self.specified_heading = specified_heading
        self.roll_setpoints = roll_setpoints
        self.pitch_setpoints = pitch_setpoints
        self.altitude_setpoints = altitude_setpoints

    def setup(self, **kwargs):
        """
        We ride on the super class (action_client) setup which creates the action client and the node
        using the same node instance we will create our service client
        """
        super().setup(**kwargs)

        self.service_client = self.node.create_client(
            srv_type=self.service_type, srv_name=self.service_name
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
            self.node.get_logger().debug("  status: {}".format(self.result_status_string))
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
        return request

    def _gen_goal(self, service_response: PoseStamped):
        output_pose = service_response.pose

        goal_msg = Locomotion.Goal()

        # Set the required fields
        goal_msg.move_rel = self.move_rel
        goal_msg.depth_rel = self.depth_rel
        goal_msg.heading_rel = self.heading_rel
        try:
            goal_msg.depth_ctrl = Locomotion.Goal.DEPTH_MODE_DEPTH
        except Exception as e:
            print(e)
            goal_msg.depth_ctrl = 0

        goal_msg.specified_heading = self.specified_heading

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
        goal_msg.roll_setpoints = self.roll_setpoints
        goal_msg.pitch_setpoints = self.pitch_setpoints
        goal_msg.altitude_setpoints = self.altitude_setpoints

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
                            self.node.resolve_service_name(self.service_name),
                            self.qualified_name,
                        )
                    )

        if not result:
            self.feedback_message = "timed out waiting for the server [{}]".format(
                self.node.resolve_service_name(self.service_name)
            )
            self.node.get_logger().error(
                "{}[{}]".format(self.feedback_message, self.qualified_name)
            )
            raise py_trees_ros.exceptions.TimedOutError(self.feedback_message)
        else:
            self.feedback_message = "... connected to service server [{}]".format(
                self.node.resolve_service_name(self.service_name)
            )
            self.node.get_logger().info("{}[{}]".format(self.feedback_message, self.qualified_name))

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


# TODO: does this even work and is it even worth to make it work
class FromConstant(FromBlackboard):
    """
    Convenience version of service client that only send the same goal.
    """

    def __init__(
        self,
        name: str,
        target_pose: PoseStamped,
        key_response: typing.Optional[str] = None,
        wait_for_server_timeout_sec: float = -3.0,
    ):
        unique_id = uuid.uuid4()
        key_request = "/goal_" + str(unique_id)
        super().__init__(
            name=name,
            key_request=key_request,
            key_response=key_response,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        )

        # parent already instantiated blackboard client
        self.blackboard.register_key(key=key_request, access=py_trees.common.Access.WRITE)
        self.blackboard.set(name=key_request, value=target_pose)
