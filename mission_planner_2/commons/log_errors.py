import py_trees
import py_trees.console as console

class LogOnFailure(py_trees.decorators.Decorator):
    """
    A decorator that logs an error message if the child node fails,
    and also stores the message in the feedback_message field.
    """
    def __init__(self, child: py_trees.behaviour.Behaviour):
        name = f"LogFailure[{child.name}]"
        super().__init__(name=name, child=child)

    def update(self):
        status = self.decorated.status

        if status == py_trees.common.Status.FAILURE:
            msg = f"[{self.decorated.name}] failed during execution."
            console.error(msg)
            self.feedback_message = msg
        else:
            self.feedback_message = f"[{self.decorated.name}] status: {status.name}"

        return status
