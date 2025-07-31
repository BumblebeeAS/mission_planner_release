import py_trees
import py_trees_ros.exceptions as exceptions
import rclpy.node
from py_trees.common import Duration
from py_trees.visitors import VisitorBase
from py_trees_ros.visitors import SetupLogger

from mission_planner_2.commons.node_registry import TreeNode


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

    def shutdown(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
        super().shutdown()
