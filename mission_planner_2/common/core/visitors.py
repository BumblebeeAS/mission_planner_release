import py_trees
from py_trees import blackboard, behaviour, display
from rclpy.node import Node
import typing

class LoggingSnapshotVisitor(py_trees.visitors.SnapshotVisitor):
    """
    Visit the tree, capturing the visited path, it's changes since the last tick.

    Additionally *logs* the snapshot to console.

    Args:
        node: The ROS2 node to obtain a logger from
        display_only_visited_behaviours: useful for cropping the unvisited part of a large tree
        display_blackboard: print to the console the relevant part of the blackboard associated with
            behaviours on the visited path
        display_activity_stream: print to the console a log of the activity on the blackboard
            over the last tick
    """
    def __init__(
        self,
        node: Node = None,
        display_only_visited_behaviours: bool = False,
        display_blackboard: bool = False,
        display_activity_stream: bool = False,
    ):
        super().__init__()
        self.node = node if node is not None else Node("tree_logger")
        self.display_only_visited_behaviours = display_only_visited_behaviours
        self.display_blackboard = display_blackboard
        self.display_activity_stream = display_activity_stream
        if self.display_activity_stream:
            blackboard.Blackboard.enable_activity_stream()

    def initialise(self) -> None:
        """Reset and initialise all variables."""
        self.root: typing.Optional[behaviour.Behaviour] = None
        super().initialise()
        if self.display_activity_stream:
            if blackboard.Blackboard.activity_stream is not None:
                blackboard.Blackboard.activity_stream.clear()

    def run(self, behaviour: behaviour.Behaviour) -> None:
        """
        Track the root of the tree and run :class:`~py_trees.visitors.SnapshotVisitor`.

        Args:
            behaviour: behaviour being visited.
        """
        self.root = behaviour  # last behaviour visited will always be the root
        super().run(behaviour)

    def finalise(self) -> None:
        """Logs a summary on /rosout after all behaviours have been visited."""
        if self.root is not None:
            self.node.get_logger().info(
                "\n"
                + display.unicode_tree(
                    root=self.root,
                    show_only_visited=self.display_only_visited_behaviours,
                    show_status=False,
                    visited=self.visited,
                    previously_visited=self.previously_visited,
                )
            )
        if self.display_blackboard:
            self.node.get_logger().info(display.unicode_blackboard(key_filter=self.visited_blackboard_keys))
        if self.display_activity_stream:
            self.node.get_logger().info(display.unicode_blackboard_activity_stream())