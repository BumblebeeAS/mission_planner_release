import py_trees
import py_trees_ros
from bb_auv_msgs.action import Grabber
from bb_behavior_msgs.action import AlignAndCollect, ControlledAscent
from bb_controls_msgs.srv import Controller
from bb_perception_msgs.action import ClusterTfAction
from bb_perception_msgs.srv import TrashToggleFrame
from py_trees_ros.subscribers import operator

from mission_planner_2.commons import checked_service
from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
)
from mission_planner_2.trees.auv.goto import goto

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

CONTROLS_SRV_TOPIC = "/auv4/controls/controller"
TOGGLE_TRASH_FRAME_CLUSTERED_TOPIC = "/auv4/trash/toggle_trash_frame_clustered"
TABLE_DETECTIONS_TOPIC = "/auv4/trash/trash/yolo/detections/on_table"
GRABBER_ACTION_TOPIC = "/auv4/actuation/grabber"
BUCKET_DETECTIONS_TOPIC = "/auv4/trash/trash/yolo/detections/in_bucket"
BASE_LINK_FRAME = "auv4/base_link_ned"
LIMITS_SERVICE_NAME = "/auv4/controls/limits"

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_TABLE_TRASH_DETECTIONS_COUNT_KEY = fk("table_trash_count_key")
_SPIN_GOTO_POSES_KEY = fk("spin_goto_poses_key")


def create_align_actuate_surface_root(
    trash_name: str,
    object_frame: str,
    command: int,
    depth_rate: float = 0.08,
    depth_tolerance: float = 0.05,
    cluster_duration: int = 10,
    z_distance: float = 0.15,
    cutoff_z_distance: float = 0.3,
    surface_depth_threshold: float = 0.05,
    align_collect_timeout_seconds: float = 90.0,
    controlled_ascent_timeout_seconds: float = 30.0,
):
    """Cluster the trash / bucket pose, then pass control to the AlignAndCollect action which
    aligns the robot to the trash / bucket and actuates the grabber to open / close. After the
    action is complete, controls remain disabled and the robot floats towards the surface. At
    a certain depth, controls are re-enabled."""
    object_frame_depth_from_table = f"{object_frame}/from_table"
    object_frame_depth_from_odom = f"{object_frame}/from_odom"
    object_frame_clustered = f"{object_frame}/clustered"

    root = py_trees.composites.Sequence(
        name=f"Align, actuate, surface ({trash_name})",
        memory=True,
    )
    cluster_trash = py_trees_ros.actions.ActionClient(
        name=f"Cluster trash ({trash_name})",
        action_type=ClusterTfAction,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=[object_frame_depth_from_table],
            out_children=[object_frame_clustered],
            duration=cluster_duration,
            use_cache=False,
        ),
    )
    srv_toggle_trash_frame_clustered = py_trees_ros.service_clients.FromConstant(
        name=f"Toggle trash frame clustered ({trash_name})",
        service_type=TrashToggleFrame,
        service_name=TOGGLE_TRASH_FRAME_CLUSTERED_TOPIC,
        service_request=TrashToggleFrame.Request(
            trash_frame_clustered=object_frame_clustered, enable=True
        ),
    )
    srv_disable_controls = checked_service.FromConstant(
        name=f"Disable controls ({trash_name})",
        service_name=CONTROLS_SRV_TOPIC,
        service_type=Controller,
        service_request=Controller.Request(
            enable=False,
            pause=False,
            disable_altitude=False,
        ),
    )
    call_trash_pickup = py_trees_ros.actions.ActionClient(
        name=f"Call trash align and collect ({trash_name})",
        action_type=AlignAndCollect,
        action_name="/auv4/align_and_collect",
        action_goal=AlignAndCollect.Goal(
            object_frame=object_frame_depth_from_odom,
            object_frame_clustered=object_frame_clustered,
            command=command,
            depth_rate=depth_rate,
            z_distance=z_distance,
            cutoff_z_distance=cutoff_z_distance,
            timeout_seconds=align_collect_timeout_seconds,
        ),
    )

    seq_surface = py_trees.composites.Sequence(
        name=f"Enable at surface ({trash_name})",
        memory=True,
    )

    action_controlled_ascent = py_trees_ros.action_clients.FromConstant(
        name="Ascent to surface",
        action_type=ControlledAscent,
        action_name="/auv4/controlled_ascent",
        action_goal=ControlledAscent.Goal(
            desired_depth=surface_depth_threshold,
            depth_tolerance=depth_tolerance,
            depth_rate=depth_rate,
            timeout_seconds=controlled_ascent_timeout_seconds,
        ),
    )
    srv_enable_controls = checked_service.FromConstant(
        name=f"Enable controls ({trash_name})",
        service_name=CONTROLS_SRV_TOPIC,
        service_type=Controller,
        service_request=Controller.Request(
            enable=True,
            pause=False,
            disable_altitude=False,
        ),
    )
    seq_surface.add_children(
        children=[
            action_controlled_ascent,
            srv_enable_controls,
        ]
    )

    root.add_children(
        children=[
            cluster_trash,
            srv_toggle_trash_frame_clustered,
            srv_disable_controls,
            call_trash_pickup,
            seq_surface,
        ]
    )

    return root


def create_checked_collection_root(
    seq_collection_root: py_trees.composites.Sequence,
    collection_result_key: str,
    trash: str,
    controlled_ascent_depth_rate: float,
    controlled_ascent_depth_tolerance: float,
    surface_depth_threshold: float,
    controlled_ascent_timeout_seconds: float,
):
    root = py_trees.composites.Selector(
        name="Checked collection root",
        memory=True,
    )

    seq_check_det = py_trees.composites.Sequence(
        name="Sequence check 0 trash on table",
        memory=True,
    )

    dynamic_count_detections = DynamicSetBlackboard(
        name="Dynamic count number of detections",
        key=collection_result_key,
        update_key=_TABLE_TRASH_DETECTIONS_COUNT_KEY,
        func=lambda results: (
            results.num_bottles_on_table
            if trash == "bottle"
            else results.num_ladles_on_table
        ),
        overwrite=True,
    )

    check_count_equals_0 = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check 0 trash on table",
        check=py_trees.common.ComparisonExpression(
            variable=_TABLE_TRASH_DETECTIONS_COUNT_KEY,
            value=0,
            operator=operator.eq,
        ),
    )

    seq_check_det.add_children(
        [
            dynamic_count_detections,
            check_count_equals_0,
        ]
    )

    seq_open_and_ascend = py_trees.composites.Sequence(
        name="Open grabber and ascend",
        memory=True,
    )

    open_grabber = py_trees_ros.action_clients.FromConstant(
        name="Open grabber",
        action_type=Grabber,
        action_name=GRABBER_ACTION_TOPIC,
        action_goal=Grabber.Goal(
            command=65535,
            tolerance=0,
            timeout_ms=5000,
        ),
    )

    force_succeed_open_grabber = py_trees.decorators.FailureIsSuccess(
        name="Force succeed open grabber",
        child=open_grabber,
    )

    action_controlled_ascent = py_trees_ros.action_clients.FromConstant(
        name="Ascent to surface",
        action_type=ControlledAscent,
        action_name="/auv4/controlled_ascent",
        action_goal=ControlledAscent.Goal(
            timeout_seconds=controlled_ascent_timeout_seconds,
            desired_depth=surface_depth_threshold,
            depth_tolerance=controlled_ascent_depth_tolerance,
            depth_rate=controlled_ascent_depth_rate,
        ),
    )

    srv_enable_controls = py_trees_ros.service_clients.FromConstant(
        name="enable controls",
        service_type=Controller,
        service_name=CONTROLS_SRV_TOPIC,
        service_request=Controller.Request(enable=True),
    )

    seq_open_and_ascend.add_children(
        [
            force_succeed_open_grabber,
            action_controlled_ascent,
            srv_enable_controls,
        ]
    )

    root.add_children(
        [
            seq_check_det,
            seq_collection_root,
            seq_open_and_ascend,
        ]
    )

    return root


def create_spin_root(collection_result_key: str):
    root = py_trees.composites.Sequence(name="Spin", memory=True)

    sel_set_spin = py_trees.composites.Selector(
        name="Select spin according to match",
        memory=True,
    )

    seq_if_match = py_trees.composites.Sequence(
        name="Spin if table + basket match",
        memory=True,
    )

    check_count_equals_4 = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check total trash = 4",
        check=py_trees.common.ComparisonExpression(
            variable=collection_result_key,
            value=4,
            operator=operator.eq,
        ),
    )

    dynamic_set_poses_match = DynamicSetBlackboard(
        name="Dynamic set goto poses (match)",
        key=collection_result_key,
        update_key=_SPIN_GOTO_POSES_KEY,
        func=lambda results: [
            create_stamped_pose(frame_id=BASE_LINK_FRAME, yaw=120.0)
            for _ in range(results.num_objects_in_bucket * 3)  # type: ignore
        ],
    )

    seq_if_match.add_children(
        [
            check_count_equals_4,
            dynamic_set_poses_match,
        ]
    )

    set_spin_poses_mismatch = py_trees.behaviours.SetBlackboardVariable(
        name="Set goto poses (mismatch)",
        variable_name=_SPIN_GOTO_POSES_KEY,
        variable_value=[
            create_stamped_pose(frame_id=BASE_LINK_FRAME, yaw=120) for _ in range(3 * 3)
        ],
        overwrite=True,
    )

    sel_set_spin.add_children(
        [
            seq_if_match,
            set_spin_poses_mismatch,
        ]
    )

    goto_spin = goto.NFromBlackboard(
        name="Goto spin",
        pose_key=_SPIN_GOTO_POSES_KEY,
        wait_between_moves_sec=0.1,
    )

    root.add_children(
        [
            sel_set_spin,
            goto_spin,
        ]
    )

    return root
