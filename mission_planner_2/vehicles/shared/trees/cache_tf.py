#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#
##############################################################################
# Description
##############################################################################

"""
Behaviour to cache a transform relationship as a pose for future navigation.

This behavior captures the spatial relationship between trajectory waypoints at a specific
moment in time, storing it as a pose that can be used later for navigation commands.
This is particularly useful when transform lookups may become inaccurate due to latency
in perception (e.g., clustering, inability to see object from new position, delays etc.).

The primary use case is to cache transform data when it's most accurate, then use the
cached pose for subsequent goto operations, decoupling navigation from real-time
transform accuracy.

Adapted from: https://github.com/splintered-reality/py_trees_ros/blob/devel/py_trees_ros/transforms.py
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import rclpy.qos
from rclpy.qos import qos_profile_system_default

##############################################################################
# Behaviours
##############################################################################
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#
##############################################################################
# Description
##############################################################################

"""
Behaviour to cache a transform relationship as a pose for future navigation.

This behavior captures the spatial relationship between trajectory waypoints at a specific
moment in time, storing it as a pose that can be used later for navigation commands.
This is particularly useful when transform lookups may become inaccurate due to latency
in perception (e.g., clustering, inability to see object from new position, delays etc.).

The primary use case is to cache transform data when it's most accurate, then use the
cached pose for subsequent goto operations, decoupling navigation from real-time
transform accuracy.

Adapted from: https://github.com/splintered-reality/py_trees_ros/blob/devel/py_trees_ros/transforms.py
"""

##############################################################################
# Imports
##############################################################################

import py_trees
import rclpy.qos
import tf2_ros
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_system_default
from tf_transformations import euler_from_quaternion

from mission_planner_2.common.util.pose_utils import create_stamped_pose

##############################################################################
# Behaviours
##############################################################################


class ToBlackboard(py_trees.behaviour.Behaviour):
    """
    Cache a transform relationship as a pose in base_link frame for future navigation.

    This behavior captures the transform between a start and end frame at a specific
    moment in time, storing it as a pose that can be used later for navigation commands.
    This is particularly useful when transform lookups may become inaccurate due to latency
    in perception (e.g., clustering, inability to see object from new position, delays etc.).

    The primary use case is to cache transform data when it's most accurate, then use the
    cached pose for subsequent goto operations, decoupling navigation from real-time
    transform accuracy.

    **Transform Convention:**

    * **start**: Target frame where robot's base_link will initially align (origin)
    * **end**: Destination frame where robot's base_link should eventually align (origin)
    * The behavior captures the end_frame to start_frame transform relationship
    * When stored in base_link coordinates, this represents the navigation vector to reach end_frame

    **Blocking Behavior:**

    If the transform lookup fails immediately, the behavior returns RUNNING status
    and writes 'None' to the blackboard until the transform becomes available.

    **Usage Patterns:**

    * clearing_policy == :attr:`~py_trees.common.ClearingPolicy.ON_INTIALISE`

    Use when subsequent behaviors need to check whether the transform was
    successfully cached before proceeding with navigation decisions.

    * clearing_policy == :attr:`~py_trees.common.ClearingPolicy.NEVER`

    Use for persistent caching across multiple tree ticks, or when managing
    your own cache invalidation based on timestamps or other criteria.

    Args:
        name: name of the behavior
        variable_name: blackboard key where the cached pose will be stored
        start: trajectory start frame name (where base_link initially aligns)
        end: trajectory end frame name (where base_link should eventually align)
        qos_profile: QoS profile for the dynamic transform subscriber
        static_qos_profile: QoS profile for static transforms (default: tf2_ros defaults)
        clearing_policy: when to clear the cached result from blackboard

    Raises:
        TypeError: if clearing_policy is :attr:`~py_trees.common.ClearingPolicy.ON_SUCCESS`
           (not valid for transform caching behaviors)

    Example:
        Cache transform relationship for navigation between inspection waypoints:

        ```python
        save_gate_left_pose = ToBlackboard(
            name="Save Gate Left Pose",
            variable_name=fk("gate_left_pose"),
            start="auv4/gate/centre",
            end="auv4/gate/left",
            qos_profile=qos_profile
        )
        ```

        Later in the tree, use cached pose:

        ```python
        goto_behavior = goto.FromBlackboard(
            pose_key=fk("gate_left_pose"),  # Retrieved from blackboard
            ...
        )
        ```
    """

    BASE_LINK_FRAME = "auv4/base_link_ned"

    def __init__(
        self,
        name: str,
        variable_name,
        start: str,
        end: str,
        qos_profile: rclpy.qos.QoSProfile = qos_profile_system_default,
        static_qos_profile: rclpy.qos.QoSProfile | None = None,
        clearing_policy: py_trees.common.ClearingPolicy = py_trees.common.ClearingPolicy.ON_INITIALISE,
    ):
        super().__init__(name=name)
        self.variable_name = variable_name
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(
            key=self.variable_name, access=py_trees.common.Access.WRITE
        )

        # flipped to look up the correct transform, see note in cfg.yaml for more details.
        self.target_frame = start
        self.source_frame = end

        self.qos_profile = qos_profile
        self.static_qos_profile = static_qos_profile
        self.clearing_policy = clearing_policy
        if self.clearing_policy == py_trees.common.ClearingPolicy.ON_SUCCESS:
            raise TypeError(
                "ON_SUCCESS is not a valid policy for transforms.ToBlackboard"
            )
        self.buffer = tf2_ros.Buffer()
        # initialise the blackboard
        self.blackboard.set(self.variable_name, None)

    def setup(self, **kwargs):
        """
        Initialize the transform listener.

        Args:
            **kwargs (:obj:`dict`): keyword arguments containing the ROS2 node
                                  required for transform listening

        Raises:
            KeyError: if 'node' key is not found in kwargs (required for tf2_ros setup)
        """
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__
            )
            raise KeyError(error_message) from e  # 'direct cause' traceability
        self.listener = tf2_ros.TransformListener(
            buffer=self.buffer,
            node=self.node,
            # spin_thread=False,
            qos=self.qos_profile,
            static_qos=self.static_qos_profile,
        )

    def initialise(self):
        """
        Reset the cached transform if using ON_INITIALISE clearing policy.

        Clears the blackboard variable (sets to 'None') when the behavior
        is initialized, ensuring fresh transform lookups on each execution.
        """
        if self.clearing_policy == py_trees.common.ClearingPolicy.ON_INITIALISE:
            self.blackboard.set(self.variable_name, None)

    def update(self):
        """
        Perform transform lookup and cache the result as a pose in base_link frame.

        Returns:
            py_trees.common.Status.SUCCESS: Transform cached successfully
            py_trees.common.Status.RUNNING: Waiting for transform to become available
        """

        class get_latest(object):
            def __init__(self):
                self.nanoseconds = 0

        if self.buffer.can_transform(
            target_frame=self.target_frame,
            source_frame=self.source_frame,
            time=Time(),
            # timeout=rclpy.duration.Duration(seconds=5),  # don't block
        ):
            stamped_transform = self.buffer.lookup_transform(
                target_frame=self.target_frame,
                source_frame=self.source_frame,
                time=Time(),
                # timeout=rclpy.duration.Duration(seconds=5)  # don't block
            )

            stamped_pose = self._tf_to_stamped_pose(stamped_transform)

            self.blackboard.set(self.variable_name, stamped_pose)
            self.feedback_message = "transformed pose saved to {}".format(
                self.variable_name
            )
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "waiting for transform"
            return py_trees.common.Status.RUNNING

    def _tf_to_stamped_pose(self, tf):
        """
        Convert a transform to a pose stamped in base_link frame.

        Takes the cached transform relationship and converts it to a pose
        that can be used directly by navigation nodes (goto behaviors).

        Args:
            tf: Transform stamped object from tf2 lookup

        Returns:
            Pose stamped in BASE_LINK_FRAME coordinates representing the
            navigation target for reaching end_frame
        """
        roll, pitch, yaw = euler_from_quaternion(
            [
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w,
            ],
        )

        return create_stamped_pose(
            frame_id=self.BASE_LINK_FRAME,
            position_x=tf.transform.translation.x,
            position_y=tf.transform.translation.y,
            position_z=tf.transform.translation.z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            use_radians=True,
        )


class ToBlackboardFromBlackboard(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        variable_name: str,
        target_frame_key: str,
        source_frame_key: str,
        qos_profile: rclpy.qos.QoSProfile = qos_profile_system_default,
        static_qos_profile: rclpy.qos.QoSProfile | None = None,
        clearing_policy: py_trees.common.ClearingPolicy = py_trees.common.ClearingPolicy.ON_INITIALISE,
    ):
        super().__init__(name=name)
        self.variable_name = variable_name
        self.blackboard = self.attach_blackboard_client(name)
        self.blackboard.register_key(
            key=self.variable_name, access=py_trees.common.Access.WRITE
        )

        self.blackboard.register_key(
            key="target",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                "/", target_frame_key
            ),
        )

        self.blackboard.register_key(
            key="source",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name(
                "/", source_frame_key
            ),
        )

        self.qos_profile = qos_profile
        self.static_qos_profile = static_qos_profile
        self.buffer = tf2_ros.Buffer()
        self.clearing_policy = clearing_policy

    def setup(self, **kwargs):
        """
        Initialize the transform listener.

        Args:
            **kwargs (:obj:`dict`): keyword arguments containing the ROS2 node
                                  required for transform listening

        Raises:
            KeyError: if 'node' key is not found in kwargs (required for tf2_ros setup)
        """
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__
            )
            raise KeyError(error_message) from e

        self.listener = tf2_ros.TransformListener(
            buffer=self.buffer,
            node=self.node,
            qos=self.qos_profile,
            static_qos=self.static_qos_profile,
        )

    def initialise(self):
        """
        Clear the blackboard variable (set to 'None') if using the
        :attr:`~py_trees.common.ClearingPolicy.ON_INTIALISE` policy.
        """
        if self.clearing_policy == py_trees.common.ClearingPolicy.ON_INITIALISE:
            self.blackboard.set(self.variable_name, None)
        self.target_frame = self.blackboard.target
        self.source_frame = self.blackboard.source

    def update(self):
        """
        Checks for the latest transform and posts it to the blackboard
        if available.
        """

        class get_latest(object):
            def __init__(self):
                self.nanoseconds = 0

        if self.buffer.can_transform(
            target_frame=self.target_frame,
            source_frame=self.source_frame,
            time=get_latest(),
            # timeout=rclpy.duration.Duration(seconds=5)  # don't block
        ):
            stamped_transform = self.buffer.lookup_transform(
                target_frame=self.target_frame,
                source_frame=self.source_frame,
                time=get_latest(),
                # timeout=rclpy.duration.Duration(seconds=5)  # don't block
            )
            self.blackboard.set(self.variable_name, stamped_transform)
            self.feedback_message = "transform saved to {}".format(self.variable_name)
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "waiting for transform from {} to {}".format(
                self.target_frame, self.source_frame
            )
            return py_trees.common.Status.RUNNING
