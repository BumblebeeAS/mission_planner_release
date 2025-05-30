"""
Common blackboard utilities for mission_planner_2.
"""

import re

import py_trees


def convert_to_safe_name(name):
    """
    Convert a string to a safe name for usage in namespaces.

    This function transforms a given name to a safe format by:
    1. Converting all characters to lowercase
    2. Replacing spaces and any non-alphanumeric characters with underscores

    Args:
        name (str): The original name to be converted

    Returns:
        str: The converted name, containing only lowercase letters, numbers, and underscores
    """
    lowercase = name.lower()
    return re.sub(r"[^a-z0-9]", "_", lowercase)


class DynamicSetBlackboard(py_trees.behaviour.Behaviour):
    """
    A non-blocking behaviour that sets a blackboard variable to a new value based on the input key.

    To use a complex custom function that takes in more inputs, wrap it as follows:
    ```python
    def your_custom_func(*args, **kwargs):
        # Do something with args and kwargs
        return new_value

    def func(x): # if one key passed in
        # Do something with curr_blackboard_value
        return your_custom_func(x, *args, **kwargs)

    def func(x, y...): # if more than one key passed in
        # Do something with curr_blackboard_value
        return your_custom_func(x, y, ..., *args, **kwargs)
    ```

    Args:
        name (str): The name of the behaviour.
        namespace (str): The namespace of the blackboard variable.
        key (str | list[str]): The key[s] of the blackboard variable to read from.
        update_key (str): The key of the blackboard variable to write to (using a key passed in will overwrite the value in the blackboard based on overwrite).
        overwrite (bool): Whether to overwrite the existing value in the blackboard.
        func (callable): A N-ary function that takes the current value and returns the new value. This function **MUST NOT**
                         be blocking or throw errors. It **MUST** return the new value to be set in the blackboard.
                         The function will be called with curr_blackboard_value[s] retrieved from the key[s] passed in.
    """

    def __init__(
        self,
        name,
        namespace,
        key,
        update_key,
        overwrite=True,
        func=lambda x: x,
    ):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(
            name="updater", namespace=namespace
        )
        self.update_key = update_key
        self.func = func
        self.overwrite = overwrite
        self.namespace = namespace
        self.keys = self._process_keys(key)
        self._register_keys(self.keys, update_key)

    def update(self) -> py_trees.common.Status:
        if self.blackboard.set(
            name=self.update_key,
            value=self._apply(),
            overwrite=self.overwrite,
        ):
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE

    def _process_keys(self, key: str | list[str]) -> list[str]:
        """
        Process the key or keys to ensure they are in a list format.
        """
        if isinstance(key, str):
            return [key]
        elif isinstance(key, list):
            return key
        else:
            raise TypeError("Key must be a string or a list of strings.")

    def _register_keys(self, keys: list[str], update_key: str) -> None:
        """Register the keys in the blackboard.

        Args:
            keys (list[str]): list of keys to read from.
            update_key (str): key to write to.
        """

        for k in keys:
            self.blackboard.register_key(key=k, access=py_trees.common.Access.READ)

        self.blackboard.register_key(
            key=update_key,
            access=py_trees.common.Access.WRITE,
        )

    def _apply(self) -> any:
        """Apply the function to the current blackboard values."""
        args = [self.blackboard.get(k) for k in self.keys]
        out = self.func(*args)
        return out
