import typing
from time import sleep

import py_trees


class SleepBehaviour(py_trees.behaviour.Behaviour):
    """A behaviour that sleeps for a specified duration.

    This node will return RUNNING until the specified sleep duration has elapsed,
    after which it returns SUCCESS. The node sleeps in small intervals to allow
    the behaviour tree to continue ticking other nodes.

    Note:
        The actual sleep duration may not map correctly to real time due to the way
        behaviour trees tick. The timing accuracy depends on the tree's tick frequency
        and other processing overhead.

    Warning:
        This node should only be used in simulation environments until further testing
        is completed for real-world applications.

    Args:
        name: The name of the behaviour.
        duration: The duration to sleep in seconds.
        interval: The interval at which to sleep in seconds. Default is 0.005 seconds.
    """

    def __init__(self, name: str, duration: float, interval: float = 0.005) -> None:
        """Initialize the sleep behaviour.

        Args:
            name: The name of the behaviour.
            duration: The duration to sleep in seconds.
            interval: The interval at which to sleep in seconds. Default is 0.005 seconds.
        """
        super(SleepBehaviour, self).__init__(name)
        self.sleep_duration = duration
        self.sleep_interval = interval
        self.elapsed_time = 0.0

    def setup(self, **kwargs: typing.Any) -> None:
        pass

    def initialise(self) -> None:
        pass

    def update(self) -> py_trees.common.Status:
        """Update the behaviour status.

        This method is called on each tick of the behaviour tree. It increments the elapsed
        time counter, sleeps for the specified interval, and returns the appropriate status.

        Returns:
            RUNNING if the sleep duration has not elapsed, SUCCESS otherwise.
        """
        if self.elapsed_time < self.sleep_duration:
            self.elapsed_time += self.sleep_interval
            sleep(self.sleep_interval)
            return py_trees.common.Status.RUNNING
        else:
            self.feedback_message = "Slept for %s seconds" % self.sleep_duration
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        pass
