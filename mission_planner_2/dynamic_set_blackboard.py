import py_trees


class DynamicSetBlackboard(py_trees.behaviour.Behaviour):
    """
    A non-blocking behaviour that sets a blackboard variable to a new value based on the input key.

    To use a complex custom function that takes in more inputs, wrap it as follows:
    ```python
    def your_custom_func(*args, **kwargs):
        # Do something with args and kwargs
        return new_value

    def func(x):
        # Do something with curr_blackboard_value
        return your_custom_func(x, *args, **kwargs)
    ```

    Args:
        name (str): The name of the behaviour.
        key (str): The key of the blackboard variable to read from.
        update_key (str): The key of the blackboard variable to write to (using the same key will overwrite the variable in the blackboard).
        overwrite (bool): Whether to overwrite the existing value in the blackboard.
        func (callable): A unary function that takes the current value and returns the new value. Thhis function **MUST NOT**
                         be blocking or throw errors. It **MUST** return the new value to be set in the blackboard.
                         The function will be called with curr_blackboard_value retrieved with the key passed in.
    """

    def __init__(
        self,
        name,
        key,
        update_key,
        overwrite=True,
        func=lambda x: x,
    ):
        super().__init__(name)
        self.key = key
        self.update_key = update_key
        self.func = func
        self.overwrite = overwrite
        self.blackboard = self.attach_blackboard_client(name="updater")
        self.blackboard.register_key(
            key=self.update_key,
            access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key=self.key,
            access=py_trees.common.Access.READ,
        )

    def update(self) -> py_trees.common.Status:
        curr = self.blackboard.get(self.key)
        if self.blackboard.set(
            name=self.update_key,
            value=self.func(curr),
            overwrite=self.overwrite,
        ):
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE
