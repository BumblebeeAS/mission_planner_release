import py_trees
import py_trees_ros
from bb_behavior_msgs.action import AlignAndCollect
from bb_controls_msgs.srv import Controller
from bb_perception_msgs.action import ClusterTfAction
from bb_perception_msgs.srv import TrashToggleFrame
from mission_planner_2.commons import checked_service
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_clustering_goal
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Float32

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_DEPTH_KEY = fk("depth")
CONTROLS_SRV_TOPIC = "/auv4/controls/controller"
TOGGLE_TRASH_FRAME_CLUSTERED_TOPIC = "/auv4/trash/toggle_trash_frame_clustered"


def create_align_actuate_surface_root(
    trash_name: str,
    object_frame: str,
    command: int,
    depth_rate: float = 0.1,
    cluster_duration: int = 10,
    z_distance: float = 0.15,
    cutoff_z_distance: float = 0.3,
    surface_depth_threshold: float = 0.2,
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
        ),
    )

    seq_surface = py_trees.composites.Sequence(
        name=f"Enable at surface ({trash_name})",
        memory=True,
    )
    sub_depth = py_trees_ros.subscribers.ToBlackboard(
        name=f"Sub depth ({trash_name})",
        topic_name="/auv4/depth",
        topic_type=Float32,
        qos_profile=qos_profile_system_default,
        blackboard_variables={_DEPTH_KEY: "data"},
    )
    # check if depth is less than or equal to threshold
    check_depth = py_trees.behaviours.CheckBlackboardVariableValue(
        name=f"Check depth ({trash_name})",
        check=py_trees.common.ComparisonExpression(
            variable=_DEPTH_KEY,
            value=surface_depth_threshold,
            operator=lambda x, y: x <= y,
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
            sub_depth,
            check_depth,
            srv_enable_controls,
        ]
    )
    # TODO: can consider more targeted retry if want
    retry_surfacing = py_trees.decorators.Retry(
        name=f"retry surfacing ({trash_name})",
        child=seq_surface,
        num_failures=1e6,
    )

    root.add_children(
        children=[
            cluster_trash,
            srv_toggle_trash_frame_clustered,
            srv_disable_controls,
            call_trash_pickup,
            retry_surfacing,
        ]
    )

    return root
