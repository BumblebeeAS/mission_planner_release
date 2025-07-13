import operator

import py_trees
import py_trees_ros
from mission_planner_2.commons.pose_utils import create_stamped_pose
from std_srvs.srv import Trigger


def create_choice_selector_root(
    choice_key: str = "choice",
    pose_key: str = "pose",
    goto_frame_key: str = "goto_frame",
    fish_bin_frame: str = "bin/fish",
    shark_bin_frame: str = "bin/shark",
) -> py_trees.composites.Selector:

    # Define nodes in execution order
    # Root selector - will try fish sequence first, then shark as fallback
    sel_choice_selector_root = py_trees.composites.Selector(
        name="Choice selector root", memory=True
    )

    # Fish sequence - executes if choice is fish
    seq_fish_setup = py_trees.composites.Sequence(name="Fish setup", memory=True)

    # Shark setup sequence
    seq_shark_setup = py_trees.composites.Sequence(name="Shark setup", memory=True)

    # Step 1: Service call to get the choice
    srv_get_choice = py_trees_ros.service_clients.FromConstant(
        name="Get choice",
        service_name="/auv4/choice/get_is_fish",
        service_type=Trigger,
        service_request=Trigger.Request(),
        key_response=choice_key,
    )

    # Step 2: Check if choice is fish
    check_is_fish = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check if fish",
        check=py_trees.common.ComparisonExpression(
            variable=choice_key,
            value=True,
            operator=lambda x, y: operator.eq(x.success, y),
        ),
    )

    # Step 3: Set fish bin pose
    set_pose_fish = py_trees.behaviours.SetBlackboardVariable(
        name="Set pose (fish)",
        variable_name=pose_key,
        variable_value=create_stamped_pose(fish_bin_frame),
        overwrite=True,
    )

    set_pose_frame_fish = py_trees.behaviours.SetBlackboardVariable(
        name="Set pose frame (fish)",
        variable_name=goto_frame_key,
        variable_value=fish_bin_frame,
        overwrite=True,
    )

    # Step 4: Set shark bin pose (fallback option)
    set_pose_shark = py_trees.behaviours.SetBlackboardVariable(
        name="Set pose (shark)",
        variable_name=pose_key,
        variable_value=create_stamped_pose(shark_bin_frame),
        overwrite=True,
    )

    set_pose_frame_shark = py_trees.behaviours.SetBlackboardVariable(
        name="Set pose frame (shark)",
        variable_name=goto_frame_key,
        variable_value=shark_bin_frame,
        overwrite=True,
    )

    # Build tree structure
    seq_fish_setup.add_children(
        [srv_get_choice, check_is_fish, set_pose_fish, set_pose_frame_fish]
    )

    seq_shark_setup.add_children([set_pose_shark, set_pose_frame_shark])

    sel_choice_selector_root.add_children([seq_fish_setup, seq_shark_setup])

    return sel_choice_selector_root
