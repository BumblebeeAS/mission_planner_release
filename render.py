import py_trees

from mission_planner_2.trees.auv.acoustics.acoustics import create_acoustics_root
from mission_planner_2.trees.auv.mother.move_to_task import create_move_to_task
from mission_planner_2.trees.auv.octagon.octagon import create_octagon_root
from mission_planner_2.trees.auv.torpedo.torpedo import create_torpedo_root

create_octagon_root
create_move_to_task
create_torpedo_root

py_trees.display.render_dot_tree(
    create_acoustics_root(),
    py_trees.common.VisibilityLevel.ALL,
    target_directory="/home/bbauv4/workspaces/ros2_ws/src/mission_planner_2",
    with_blackboard_variables=False,
)
