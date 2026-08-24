import functools

import py_trees
import py_trees_ros.exceptions as exceptions
import rclpy.node
from py_trees.common import Duration
from py_trees.visitors import VisitorBase
from py_trees_ros.visitors import SetupLogger

# task3: import LoggingSnapshotVisitor
from mission_planner_2.common.core.visitors import LoggingSnapshotVisitor, StructuredSnapshotVisitor

from mission_planner_2.common.core.tree_node import TreeNode


class BumbleTree(py_trees.trees.BehaviourTree):

    def __init__(self, root: py_trees.behaviour.Behaviour):
        """
        Initializes the BumbleTree with a root behavior.

        :param root: The root behaviour of the tree.
        """
        super().__init__(root=root)

    def setup(
        self,
        timeout: float | Duration = py_trees.common.Duration.INFINITE,
        visitor: VisitorBase | None = None,
        node: rclpy.node.Node | None = None,
        **kwargs: int,
    ) -> None:
        """Setup the behavior tree.

        Args:
            timeout (float | Duration, optional): The maximum time to wait for setup to complete. Defaults to py_trees.common.Duration.INFINITE.
            visitor (VisitorBase | None, optional): A visitor for the setup of the tree. Defaults to None.
            node (rclpy.node.Node | None, optional): The ROS 2 node to use for the tree. Defaults to TreeNode.

        Raises:
            ValueError: If a rclpy.node.Node instance is not provided in kwargs.
            exceptions.TimedOutError: If the setup times out.
        """
        if node is None:
            node = TreeNode()

        self.node = node

        if visitor is None:
            visitor = SetupLogger(node=node)

        # task3: add LoggingSnapshotVisitor, StructuredSnapshotVisitor to tree
        log_visitor = LoggingSnapshotVisitor(
            node=node,
            display_only_visited_behaviours=True,
            display_blackboard=False,
            display_activity_stream=True,
        )
        self.add_visitor(log_visitor)
        structured_visitor = StructuredSnapshotVisitor(
            node=node,
            display_only_visited_behaviours=True,
            display_blackboard=False,
            display_activity_stream=True,
        )
        self.add_visitor(structured_visitor)

        try:
            super().setup(
                timeout=timeout,
                visitor=visitor,
                node=self.node,
                **kwargs,
            )
        except RuntimeError as e:
            if str(e) == "tree setup interrupted or timed out":
                raise exceptions.TimedOutError(str(e))
            else:
                raise

    def shutdown(self):
        """
        Cleanly shut down rclpy timers and nodes.
        """
        # stop ticking if we're ticking
        if self.node is not None:
            if self.timer is not None:
                self.timer.cancel()
                self.node.destroy_timer(self.timer)
        # call shutdown on each behaviour first, in case it has
        # some esoteric shutdown steps
        super().shutdown()
        if self.node is not None:
            # shutdown the node - this *should* automagically clean
            # up any non-estoeric shutdown of ros communications
            # inside behaviours
            self.node.destroy_node()

    def tick_tock(
        self,
        period_ms,
        number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
        pre_tick_handler=None,
        post_tick_handler=None,
    ):
        self.timer = self.node.create_timer(
            period_ms / 1000.0,  # unit 'seconds'
            functools.partial(
                self._tick_tock_timer_callback,
                number_of_iterations=number_of_iterations,
                pre_tick_handler=pre_tick_handler,
                post_tick_handler=post_tick_handler,
            ),
        )
        self.tick_tock_count = 0

    def _tick_tock_timer_callback(
        self, number_of_iterations, pre_tick_handler, post_tick_handler
    ):
        if (
            number_of_iterations == py_trees.trees.CONTINUOUS_TICK_TOCK
            or self.tick_tock_count < number_of_iterations
        ):
            self.tick(pre_tick_handler, post_tick_handler)
            self.tick_tock_count += 1
        else:
            self.timer.cancel()
