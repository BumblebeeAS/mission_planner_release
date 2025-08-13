import py_trees

from mission_planner_2.trees.auv.acoustics.order_by_ping import create_order_by_ping_root


def create_acoustics_root(
    first_task_name: str,
    second_task_name: str,
    first_task: list[py_trees.behaviour.Behaviour],
    second_task: list[py_trees.behaviour.Behaviour]
) -> py_trees.behaviour.Behaviour:

    seq_first_task = py_trees.composites.Sequence(
        name=f"Sequence {first_task_name}",
        memory=True
    )
    
    seq_second_task = py_trees.composites.Sequence(
        name=f"Sequence {second_task_name}",
        memory=True
    )

    seq_first_task.add_children(
        children=first_task
    )

    seq_second_task.add_children(
        children=second_task
    )

    return create_order_by_ping_root(
        lambda: seq_first_task,
        lambda: seq_second_task
    )
