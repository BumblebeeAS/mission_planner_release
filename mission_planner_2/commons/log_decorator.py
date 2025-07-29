from typing import Any

import py_trees
import py_trees.display
import rclpy.node


class RosLoggerDecorator(py_trees.decorators.Decorator):
    """
    RosLoggerDecorator that logs the tree structure when a specific status is reached.
    """

    def __init__(
        self,
        name: str,
        child: py_trees.behaviour.Behaviour,
        log_tree_on_status: py_trees.common.Status = py_trees.common.Status.RUNNING,
    ):
        super().__init__(name, child)
        self.log_tree_on_status = log_tree_on_status
        self._root = None

    def setup(self, **kwargs: Any) -> None:
        super().setup(**kwargs)

        try:
            node: rclpy.node.Node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__
            )
            raise KeyError(error_message) from e

        self.logger = node.get_logger()

    def update(self) -> py_trees.common.Status:
        status = self.decorated.status

        if self.log_tree_on_status == status:
            # TODO: is this the correct display function to use ask advay
            tree_str = py_trees.display.unicode_tree(
                root=self.decorated, show_status=True, show_only_visited=False
            )
            self.logger.info(f"Tree state for {self.decorated.name}:\n{tree_str}")

        return status
