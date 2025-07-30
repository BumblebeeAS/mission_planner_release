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
        log_level: str,
    ):
        super().__init__(name, child)
        self.log_level = log_level
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

        # TODO: is this the correct display function to use ask advay
        tree_str = py_trees.display.unicode_tree(
            root=self.decorated, show_status=True, show_only_visited=False
        )

        msg = f"Tree state for {self.decorated.name}:\n{tree_str}"
        if self.log_level == "info":
            self.logger.info(msg)
        elif self.log_level == "debug":
            self.logger.debug(msg)
        elif self.log_level == "error":
            self.logger.error(msg)
        elif self.log_level == "warn":
            self.logger.warn(msg)
        else:
            raise ValueError(f"loglevel unknown {self.log_level}")

        return status
