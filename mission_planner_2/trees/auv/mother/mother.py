import py_trees
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.pose_utils import (
    compute_start_to_end_vector,
    create_stamped_pose,
)
from mission_planner_2.commons.tf_checker import create_tf_checker_from_constant_root
from mission_planner_2.trees.auv.bins.bins import create_bin_root
from mission_planner_2.trees.auv.button.wait_for_button import (
    create_wait_for_button_root,
)
from mission_planner_2.trees.auv.gate.gate import create_gate_root
from mission_planner_2.trees.auv.gate.move_to_task import create_move_to_gate_task_root
from mission_planner_2.trees.auv.gate.return_home import create_return_root
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.octagon.octagon import create_octagon_root
from mission_planner_2.trees.auv.slalom.slalom import create_slalom_root
from mission_planner_2.trees.auv.torpedo.torpedo import create_torpedo_root

BUTTON_TOPIC = "/auv4/button/left"
IS_LEFT_KEY = "/global/is_left_side"  # Global key for left option or not
BASE_LINK_KEY = "/global/base_link"
WORLD_KEY = "/global/world"
CURRENT_ODOM_KEY = "/global/current_odom"
ZERO_YAW_POSE_KEY = "/global/zero_yaw_pose_key"


MAP_NED_COORDS_GATE_START = {
    "x": 2.0,
    "y": 0.0,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
MAP_NED_COORDS_GATE_END = {  # TODO: MUST TUNE
    "x": 5.0,
    "y": 0.0,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
MAP_NED_COORDS_SLALOM_START = {
    "x": 5.0,
    "y": 0.0,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
MAP_NED_COORDS_SLALOM_END = {  # TODO: MUST TUNE
    "x": 12.0,
    "y": -1.5,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
MAP_NED_COORDS_BIN = {
    "x": 15.0,
    "y": -1.5,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
MAP_NED_COORDS_TORPEDO = {
    "x": 15.0,
    "y": 1.5,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}
MAP_NED_COORDS_OCTAGON = {
    "x": 23.0,
    "y": -1.0,
    "z": 0.3,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
}


def create_move_to_task(task: str, start: dict, end: dict, stabilise_time: float = 5.0):
    root = py_trees.composites.Sequence(
        name=f"Move to {task}",
        memory=True,
    )

    get_odom = create_tf_checker_from_constant_root(
        start_frames=["world_ned"],
        end_frames=["auv4/base_link_ned"],
        update_keys=[CURRENT_ODOM_KEY],
        fallback_val=[None],
    )

    dynamic_create_zero_yaw_pose = DynamicSetBlackboard(
        name="Set zero yaw pose",
        key=CURRENT_ODOM_KEY,
        update_key=ZERO_YAW_POSE_KEY,
        overwrite=True,
        func=get_zero_yaw_pose,
    )

    goto_zero_yaw = goto.FromBlackboard(
        name="Goto zero yaw",
        pose_key=ZERO_YAW_POSE_KEY,
    )

    timer_stabilise = py_trees.timers.Timer(
        name="Stabilise",
        duration=stabilise_time,
    )

    coords = compute_start_to_end_vector(start, end)

    task_pose = create_stamped_pose(
        "auv4/base_link_ned",
        position_x=coords["x"],
        position_y=coords["y"],
        position_z=coords["z"],
        roll=coords["roll"],
        pitch=coords["pitch"],
        yaw=coords["yaw"],
    )

    goto_task = goto.FromConstant(
        name=f"Goto {task} start",
        pose=task_pose,
    )

    root.add_children(
        [
            get_odom,
            dynamic_create_zero_yaw_pose,
            goto_zero_yaw,
            timer_stabilise,
            goto_task,
        ]
    )

    return root


def get_zero_yaw_pose(tf: TransformStamped):
    _, _, y = euler_from_quaternion(
        [
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ]
    )

    return create_stamped_pose(
        frame_id="auv4/base_link_ned",
        yaw=-y,
        use_radians=True,
    )


def create_mother():
    root = py_trees.composites.Sequence(
        name="mother",
        memory=True,
    )

    wait_for_button = create_wait_for_button_root(
        button_topic=BUTTON_TOPIC,
        num_retries=1000000,
    )

    set_base_link_frame = py_trees.behaviours.SetBlackboardVariable(
        name="Set Base Link Frame",
        variable_name=BASE_LINK_KEY,
        variable_value="auv4/base_link_ned",
        overwrite=True,
    )

    set_world_frame = py_trees.behaviours.SetBlackboardVariable(
        name="Set World Frame",
        variable_name=WORLD_KEY,
        variable_value="world_ned",
        overwrite=True,
    )

    gate_root = create_gate_root()
    move_to_gate = create_move_to_gate_task_root(MAP_NED_COORDS_GATE_START)

    move_to_slalom = create_move_to_task(
        task="slalom",
        start=MAP_NED_COORDS_GATE_END,
        end=MAP_NED_COORDS_SLALOM_START,
    )
    slalom_root = create_slalom_root()

    move_to_bin = create_move_to_task(
        task="bin",
        start=MAP_NED_COORDS_SLALOM_END,
        end=MAP_NED_COORDS_BIN,
    )
    bin_root = create_bin_root()

    move_to_torpedo = create_move_to_task(
        task="torpedo",
        start=MAP_NED_COORDS_BIN,
        end=MAP_NED_COORDS_TORPEDO,
    )
    torpedo_root = create_torpedo_root()

    move_to_octagon = create_move_to_task(
        task="octagon",
        start=MAP_NED_COORDS_TORPEDO,
        end=MAP_NED_COORDS_OCTAGON,
    )
    octagon_root = create_octagon_root()

    # TODO: see if need a move to gate here to go closer to do the return task
    return_root = create_return_root()

    # TODO: PURELY FOR TESTING
    set_is_left = py_trees.behaviours.SetBlackboardVariable(
        name="Set is left for test",
        variable_name=IS_LEFT_KEY,
        variable_value=True,
        overwrite=True,
    )

    root.add_children(
        [
            # wait_for_button,
            # set_is_left,
            # set_base_link_frame,
            # set_world_frame,  # TODO: use multi set bb?
            # move_to_gate,
            # gate_root,
            # move_to_slalom,
            # slalom_root,
            # move_to_bin,
            # bin_root,
            # move_to_torpedo,
            # torpedo_root,
            # move_to_octagon,
            octagon_root,
            # return_root,
        ]
    )

    return root
