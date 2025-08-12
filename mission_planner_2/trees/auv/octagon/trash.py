import py_trees
import py_trees_ros
from bb_auv_msgs.action import Grabber
from bb_behavior_msgs.action import AlignAndCollect, ControlledAscent
from bb_controls_msgs.srv import Controller
from bb_perception_msgs.srv import GetObjectCount, TrashToggleFrame
from geometry_msgs.msg import PoseStamped, TransformStamped
from py_trees_ros.subscribers import operator
from tf_transformations import euler_from_quaternion

from mission_planner_2.commons import checked_service, shared_action_client
from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.node_registry import SharedAction
from mission_planner_2.commons.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
)
from mission_planner_2.commons.tf_checker import create_tf_checker_from_constant_root
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.octagon.helpers import create_spin_goal

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

CONTROLS_SRV_TOPIC = "/auv4/controls/controller"
TOGGLE_TRASH_FRAME_CLUSTERED_TOPIC = "/auv4/trash/toggle_trash_frame_clustered"
TABLE_DETECTIONS_TOPIC = "/auv4/trash/trash/yolo/detections/on_table"
GRABBER_ACTION_TOPIC = "/auv4/actuation/grabber"
BUCKET_DETECTIONS_TOPIC = "/auv4/trash/trash/yolo/detections/in_bucket"
BASE_LINK_FRAME = "auv4/base_link_ned"
LIMITS_SERVICE_NAME = "/auv4/controls/limits"
BOT_CAM_FRAME = "auv4/bot_cam_optical"

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_TABLE_TRASH_DETECTIONS_COUNT_KEY = fk("table_trash_count_key")


def create_align_actuate_surface_root(
    trash_name: str,
    object_frame: str,
    command: int,
    initial_collection_result_key: str,
    trash_count_service: str,
    depth_rate: float = 0.08,
    depth_tolerance: float = 0.05,
    cluster_duration: int = 10,
    xy_distance_threshold: float = 0.05,
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

    par_cluster_and_count = py_trees.composites.Parallel(
        name="Cluster basket (and check for changed count if dropping)",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )

    cluster_trash = shared_action_client.FromConstant(
        name=f"Cluster trash ({trash_name})",
        shared_action=SharedAction.CLUSTER,
        action_goal=create_clustering_goal(
            in_children=[object_frame_depth_from_table],
            out_children=[object_frame_clustered],
            duration=cluster_duration,
            use_cache=False,
        ),
    )

    # if dropping
    if command == AlignAndCollect.Goal.OPEN:
        check_collections_changed = create_check_collections_changed_root(
            initial_collection_result_key=initial_collection_result_key,
            trash_count_service=trash_count_service,
            collect_duration=cluster_duration,
        )

        par_children = [
            cluster_trash,
            check_collections_changed,
        ]
    else:
        par_children = [
            cluster_trash,
        ]

    par_cluster_and_count.add_children(par_children)  # type: ignore

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

    call_trash_pickup = shared_action_client.FromConstant(
        name=f"Call trash align and collect ({trash_name})",
        shared_action=SharedAction.ALIGN_AND_COLLECT,
        action_goal=AlignAndCollect.Goal(
            object_frame=object_frame_depth_from_odom,
            object_frame_clustered=object_frame_clustered,
            command=command,
            depth_rate=depth_rate,
            xy_distance_threshold=xy_distance_threshold,
            z_distance=z_distance,
            cutoff_z_distance=cutoff_z_distance,
            timeout_seconds=align_collect_timeout_seconds,
        ),
    )

    seq_surface = py_trees.composites.Sequence(
        name=f"Enable at surface ({trash_name})",
        memory=True,
    )

    action_controlled_ascent = shared_action_client.FromConstant(
        name="Ascend to surface",
        shared_action=SharedAction.CONTROLLED_ASCENT,
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
            par_cluster_and_count,
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

    seq_open_and_ascend = create_open_and_ascend_root(
        grabber_action_topic=GRABBER_ACTION_TOPIC,
        controls_srv_topic=CONTROLS_SRV_TOPIC,
        controlled_ascent_timeout_seconds=controlled_ascent_timeout_seconds,
        surface_depth_threshold=surface_depth_threshold,
        controlled_ascent_depth_tolerance=controlled_ascent_depth_tolerance,
        controlled_ascent_depth_rate=controlled_ascent_depth_rate,
    )

    root.add_children(
        [
            seq_check_det,
            seq_collection_root,
            seq_open_and_ascend,
        ]
    )

    return root


def create_open_and_ascend_root(
    grabber_action_topic: str,
    controls_srv_topic: str,
    controlled_ascent_timeout_seconds: float,
    surface_depth_threshold: float,
    controlled_ascent_depth_tolerance: float,
    controlled_ascent_depth_rate: float,
    open_grabber_first: bool = True,
):
    """
    This is a fallback!!!
    """

    seq_open_and_ascend_and_enable = py_trees.composites.Sequence(
        name="Open grabber and ascend and enable",
        memory=True,
    )

    seq_open_and_ascend = py_trees.composites.Sequence(
        name="Open grabber and ascend",
        memory=True,
    )

    open_grabber = shared_action_client.FromConstant(
        name="Open grabber",
        shared_action=SharedAction.GRABBER,
        action_goal=Grabber.Goal(
            command=65535,
            tolerance=0,
            timeout_ms=5000,
        ),
    )

    action_controlled_ascent = shared_action_client.FromConstant(
        name="Ascent to surface",
        shared_action=SharedAction.CONTROLLED_ASCENT,
        action_goal=ControlledAscent.Goal(
            timeout_seconds=controlled_ascent_timeout_seconds,
            desired_depth=surface_depth_threshold,
            depth_tolerance=controlled_ascent_depth_tolerance,
            depth_rate=controlled_ascent_depth_rate,
        ),
    )

    if open_grabber_first:
        children = [
            open_grabber,
            action_controlled_ascent,
        ]
    else:
        children = [
            action_controlled_ascent,
            open_grabber,
        ]

    seq_open_and_ascend.add_children(children=children)

    force_succeed_seq_open_and_ascend = py_trees.decorators.FailureIsSuccess(
        name="Force succeed sequence open and ascend",
        child=seq_open_and_ascend,
    )

    srv_enable_controls = py_trees_ros.service_clients.FromConstant(
        name="Enable controls",
        service_type=Controller,
        service_name=controls_srv_topic,
        service_request=Controller.Request(enable=True),
    )

    seq_open_and_ascend_and_enable.add_children(
        [
            force_succeed_seq_open_and_ascend,
            srv_enable_controls,
        ]
    )

    force_succeed_seq_open_and_ascend_and_enable = py_trees.decorators.FailureIsSuccess(
        name="Force succeed sequence open and ascend and enable",
        child=seq_open_and_ascend_and_enable,
    )

    return force_succeed_seq_open_and_ascend_and_enable


def create_goto_table_centre_root(
    table_centre_frame: str,
    table_centre_frame_clustered: str,
    cluster_duration: int,
    table_cluster_failure_count_key: str,
    is_grabber_open: bool,
    trash: str | None,
    depth_override: float | None = None,
):
    if trash == "bottle":
        goto_pose = create_stamped_pose(
            frame_id=table_centre_frame_clustered,
            yaw=90.0,
        )
    elif trash == "ladle":
        goto_pose = create_stamped_pose(
            frame_id=table_centre_frame_clustered, yaw=-90.0
        )
    elif trash is None:
        goto_pose = create_stamped_pose(
            frame_id=table_centre_frame_clustered
        )  # should never be used
    else:
        raise ValueError("wee")

    root = py_trees.composites.Sequence(
        name="Goto table centre",
        memory=True,
    )

    sel_cluster_centre_with_failure_count = py_trees.composites.Selector(
        name="Cluster table centre with failure count", memory=True
    )

    seq_cluster_table_centre = py_trees.composites.Sequence(
        name="Cluster table centre and reset count",
        memory=True,
    )

    cluster_table_centre = shared_action_client.FromConstant(
        name="Cluster centre",
        shared_action=SharedAction.CLUSTER,
        action_goal=create_clustering_goal(
            in_children=table_centre_frame,
            out_children=table_centre_frame_clustered,
            duration=cluster_duration,
            use_cache=False,
        ),
    )

    reset_failure_count = DynamicSetBlackboard(
        name="Reset table clustering failure count",
        key=table_cluster_failure_count_key,
        update_key=table_cluster_failure_count_key,
        overwrite=True,
        func=lambda x: 0,
    )

    seq_cluster_table_centre.add_children(
        [
            cluster_table_centre,
            reset_failure_count,
        ]
    )

    update_failure_count = DynamicSetBlackboard(
        name="Update table clustering failure count",
        key=table_cluster_failure_count_key,
        update_key=table_cluster_failure_count_key,
        overwrite=True,
        func=lambda x: x + 1,
    )

    force_fail_update_failure_count = py_trees.decorators.SuccessIsFailure(
        name="Force fail update failure count",
        child=update_failure_count,
    )

    if is_grabber_open:
        selector_children = [
            seq_cluster_table_centre,
            force_fail_update_failure_count,
        ]
    else:
        selector_children = [
            seq_cluster_table_centre,
        ]

    sel_cluster_centre_with_failure_count.add_children(selector_children)  # type: ignore

    goto_table_centre_with_bucket = goto.FromConstant(
        name="Goto table centre",
        pose=goto_pose,
        ignore_depth=True,
        anchor_frame_name=BOT_CAM_FRAME,
    )

    _base_link_to_table_centre_key = fk("base_link_to_table_centre_tf")
    _zero_yaw_pose_key = fk("zero_yaw_pose")

    lookup_base_link_to_table_centre = create_tf_checker_from_constant_root(
        start_frames=[BASE_LINK_FRAME],
        end_frames=[table_centre_frame_clustered],
        timeout=5.0,
        update_keys=[_base_link_to_table_centre_key],
        fallback_val=[None],
    )

    create_zeroed_yaw_pose = DynamicSetBlackboard(
        name="Create zeroed yaw pose",
        key=_base_link_to_table_centre_key,
        update_key=_zero_yaw_pose_key,
        overwrite=True,
        func=get_zeroed_yaw_pose,
    )

    if depth_override is not None:
        goto_table_centre = goto.FromBlackboard(
            name="Goto table centre",
            pose_key=_zero_yaw_pose_key,
            anchor_frame_name=BOT_CAM_FRAME,
            depth_override_value=depth_override,
        )
    else:
        goto_table_centre = goto.FromBlackboard(
            name="Goto table centre",
            pose_key=_zero_yaw_pose_key,
            ignore_depth=True,
            anchor_frame_name=BOT_CAM_FRAME,
        )

    if trash is None:
        root_children = [
            sel_cluster_centre_with_failure_count,
            lookup_base_link_to_table_centre,
            create_zeroed_yaw_pose,
            goto_table_centre,
        ]
    else:
        root_children = [
            sel_cluster_centre_with_failure_count,
            goto_table_centre_with_bucket,
        ]

    root.add_children(root_children)

    return root


def get_zeroed_yaw_pose(tf: TransformStamped) -> PoseStamped:
    r, p, y = euler_from_quaternion(
        [
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ]
    )

    return create_stamped_pose(
        frame_id=BASE_LINK_FRAME,
        position_x=tf.transform.translation.x,
        position_y=tf.transform.translation.y,
        position_z=tf.transform.translation.z,
        roll=r,
        pitch=p,
        yaw=0.0,
        use_radians=True,
    )


def create_count_table_root(
    collection_result_key: str,
    trash_count_service: str,
    collect_duration: float,
):
    root = py_trees.composites.Sequence(
        name="Count trash sequence",
        memory=True,
    )

    srv_activate_count = py_trees_ros.service_clients.FromConstant(
        name="Activate trash count service",
        service_type=GetObjectCount,
        service_name=trash_count_service,
        service_request=GetObjectCount.Request(
            enable=True,
        ),
    )

    timer_wait_for_collection = py_trees.timers.Timer(
        name="Wait for data collection",
        duration=collect_duration,
    )

    srv_deactivate_count = py_trees_ros.service_clients.FromConstant(
        name="Deactivate trash count service",
        service_type=GetObjectCount,
        service_name=trash_count_service,
        service_request=GetObjectCount.Request(
            enable=False,
        ),
        key_response=collection_result_key,
    )

    root.add_children(
        [
            srv_activate_count,
            timer_wait_for_collection,
            srv_deactivate_count,
        ]
    )

    return root


def create_check_collections_changed_root(
    initial_collection_result_key: str,
    trash_count_service: str,
    collect_duration: float,
):
    new_collect_key = fk("new_collections")
    is_changed_key = fk("is_count_table_changed")

    root = py_trees.composites.Sequence(
        name="Check table count changed seq",
        memory=True,
    )

    seq_count = create_count_table_root(
        collection_result_key=new_collect_key,
        trash_count_service=trash_count_service,
        collect_duration=collect_duration,
    )

    dynamic_check_table_changed = DynamicSetBlackboard(
        name="Check table count drop",
        key=[initial_collection_result_key, new_collect_key],
        update_key=is_changed_key,
        func=lambda initial, after: (
            after.num_bottles_on_table + after.num_ladles_on_table
            < initial.num_bottles_on_table + initial.num_ladles_on_table
        ),
    )

    check_is_changed = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Check table count dropped",
        check=py_trees.common.ComparisonExpression(
            variable=is_changed_key, value=True, operator=operator.eq
        ),
    )

    root.add_children(
        [
            seq_count,
            dynamic_check_table_changed,
            check_is_changed,
        ]
    )

    return root


def create_trash_count_collection_root(
    collection_result_key: str,
    trash_count_service: str,
    table_centre_frame: str,
    table_centre_frame_clustered: str,
    table_cluster_failure_count_key: str,
    cluster_duration: int,
    collect_duration: float,
    trash: str | None,
    is_grabber_open: bool,
):
    root = py_trees.composites.Sequence(
        name="Get trash counts sequence",
        memory=True,
    )

    goto_table_centre = create_goto_table_centre_root(
        trash=trash,
        table_centre_frame=table_centre_frame,
        table_centre_frame_clustered=table_centre_frame_clustered,
        table_cluster_failure_count_key=table_cluster_failure_count_key,
        cluster_duration=cluster_duration,
        is_grabber_open=is_grabber_open,
    )

    seq_count = create_count_table_root(
        collection_result_key=collection_result_key,
        trash_count_service=trash_count_service,
        collect_duration=collect_duration,
    )

    root.add_children(
        [
            goto_table_centre,
            seq_count,
        ]
    )

    return root


def create_spin_root(
    collection_result_key: str,
    spin_action_goal_key: str,
    trash_count_service: str,
    table_centre_frame: str,
    table_centre_frame_clustered: str,
    table_cluster_failure_count_key: str,
    cluster_duration: int,
    collect_duration: float,
    controlled_spin_topic: str,
):
    root = py_trees.composites.Sequence(name="Spin", memory=True)

    collect_trash_counts = create_trash_count_collection_root(
        collection_result_key=collection_result_key,
        trash_count_service=trash_count_service,
        trash=None,
        table_centre_frame=table_centre_frame,
        table_centre_frame_clustered=table_centre_frame_clustered,
        table_cluster_failure_count_key=table_cluster_failure_count_key,
        cluster_duration=cluster_duration,
        collect_duration=collect_duration,
        is_grabber_open=True,
    )

    set_spin_action_goal = DynamicSetBlackboard(
        name="Set spin action goal",
        key=collection_result_key,
        update_key=spin_action_goal_key,
        overwrite=True,
        func=create_spin_goal,
    )

    spin = shared_action_client.FromBlackboard(
        name="Call controlled spin",
        shared_action=SharedAction.CONTROLLED_SPIN,
        key=spin_action_goal_key,
    )

    srv_disable_controls = checked_service.FromConstant(
        name="Disable controls (for spin)",
        service_name=CONTROLS_SRV_TOPIC,
        service_type=Controller,
        service_request=Controller.Request(
            enable=False,
            pause=False,
            disable_altitude=False,
        ),
    )

    srv_enable_controls = checked_service.FromConstant(
        name="Enable controls (for spin)",
        service_name=CONTROLS_SRV_TOPIC,
        service_type=Controller,
        service_request=Controller.Request(
            enable=True,
            pause=False,
            disable_altitude=False,
        ),
    )

    root.add_children(
        [
            collect_trash_counts,
            set_spin_action_goal,
            srv_disable_controls,
            spin,
            srv_enable_controls,
        ]
    )

    return root
