import py_trees


class DynamicSetBlackboard(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name,
        key,
        update_key,
        overwrite=False,
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
