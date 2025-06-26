import py_trees


def create_dummy_root():
    """This was created for generating meaningless trees to display on the website."""

    sequence_tasks = py_trees.composites.Sequence(
        name="sequence tasks",
        memory=True
    )

    set_option = py_trees.behaviours.SetBlackboardVariable(
        name="set blackboard is option a",
        variable_name="option/is_a",
        variable_value=True,
        overwrite=True
    )

    selector_a_or_b = py_trees.composites.Selector(
        name="select a or b",
        memory=True
    )

    sequence_option_a = py_trees.composites.Sequence(
        name="sequence option a",
        memory=True
    )

    check_is_a = py_trees.behaviours.CheckBlackboardVariableValue(
        name="check is option a",
        check=py_trees.common.ComparisonExpression(
            variable="option/is_a",
            value=True,
            operator=bool.__eq__
        )
    )

    dummy_a = py_trees.behaviours.Dummy(
        name="go to a"
    )

    dummy_b = py_trees.behaviours.Dummy(
        name="go to b"
    )

    decorator_timeout = py_trees.decorators.Timeout(
        name="timeout 5s",
        child=py_trees.behaviours.Dummy("activate manipulator"),
        duration=5.0
    )

    sequence_option_a.add_children(
        children=[
            check_is_a,
            dummy_a
        ]
    )

    selector_a_or_b.add_children(
        children=[
            sequence_option_a,
            dummy_b
        ]
    )

    sequence_tasks.add_children(
        children=[
            set_option,
            selector_a_or_b,
            decorator_timeout
        ]
    )

    return sequence_tasks
