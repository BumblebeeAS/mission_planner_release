import py_trees
import py_trees_ros
from bb_controls_msgs.srv import Controller
from bb_perception_msgs.action import ClusterTf
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Float32

from mission_planner_2.commons import checked_service
from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_clustering_goal
from mission_planner_2.commons.tf_checker import create_tf_checker_from_constant_root
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.octagon.helpers import view_frame_func

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_DEPTH_KEY = fk("depth")
_FISH_TF_KEY = fk("fish_tf")
_SHARK_TF_KEY = fk("shark_tf")
_GO_SURFACE_FRAME_KEY = fk("surface_frame")


def create_rubbish_root(
    depth_threshold: float = 0.1,
    rubbish_frame: str = "bottle",
    rubbish_frame_clustered: str = "bottle/clustered",
    cluster_duration: int = 10,
    call_samuel: py_trees_ros.actions.ActionClient = None,
    rubbish_name: str = "rubbish",
):
    root = py_trees.composites.Sequence(
        name=f"Rubbish ({rubbish_name})",
        memory=True,
    )

    cluster_rubbish = py_trees_ros.actions.ActionClient(
        name=f"Cluster rubbish ({rubbish_name})",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=[rubbish_frame],
            out_children=[rubbish_frame_clustered],
            duration=cluster_duration,
            use_cache=False,
        ),
    )

    srv_disable_controls = checked_service.FromConstant(
        name=f"Disable controls ({rubbish_name})",
        service_name="/auv4/controls/disable",
        service_type=Controller,
        service_request=Controller.Request(
            enable=False,
            pause=False,
            disable_altitude=False,
        ),
    )

    seq_surface = py_trees.composites.Sequence(
        name=f"Enable at surface ({rubbish_name})",
        memory=True,
    )

    sub_depth = py_trees_ros.subscribers.ToBlackboard(
        name=f"Sub depth ({rubbish_name})",
        topic_name="/auv4/depth",
        topic_type=Float32,
        qos_profile=qos_profile_system_default,
        blackboard_variables={_DEPTH_KEY: "data"},
    )

    # check if depth is less than or equal to threshold
    check_depth = py_trees.behaviours.CheckBlackboardVariableValue(
        name=f"Check depth ({rubbish_name})",
        check=py_trees.common.ComparisonExpression(
            variable=_DEPTH_KEY,
            value=depth_threshold,
            operator=lambda x, y: x <= y,
        ),
    )

    srv_enable_controls = checked_service.FromConstant(
        name=f"Enable controls ({rubbish_name})",
        service_name="/auv4/controls/enable",
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

    root.add_children(
        children=[
            cluster_rubbish,
            srv_disable_controls,
            call_samuel,
            py_trees.decorators.Retry(  # TODO: can consider more targeted retry if want
                name=f"retry surfacing ({rubbish_name})",
                child=seq_surface,
                num_failures=1e6,
            ),
        ]
    )

    return root


def create_reset_after_rubbish_root(
    fish_frame: str = "fish",
    fish_frame_clustered: str = "fish/clustered",
    shark_frame: str = "shark",
    shark_frame_clustered: str = "shark/clustered",
    fish_view_frame: str = "fish_view",
    fish_view_frame_hardcoded: str = "fish_view_hardcoded",
    shark_view_frame: str = "shark_view",
    shark_view_frame_hardcoded: str = "shark_view_hardcoded",
    cluster_duration: int = 10,
    choice_key: str = "choice",
    rubbish_name: str = "spoon 1",
):
    root = py_trees.composites.Sequence(
        name=f"Reset after rubbish ({rubbish_name})",
        memory=True,
    )

    # TODO: add the search seq here a bit laze now can be done later

    cluster_reset_symbols_1 = py_trees_ros.actions.ActionClient(
        name=f"Cluster symbols ({rubbish_name})",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=[fish_frame, shark_frame],
            out_children=[fish_frame_clustered, shark_frame_clustered],
            duration=cluster_duration,
            use_cache=False,
            persistent=True,
        ),
    )

    symbol_tf_checker = create_tf_checker_from_constant_root(
        start_frames=[fish_frame_clustered, shark_frame_clustered],
        update_keys=[_FISH_TF_KEY, _SHARK_TF_KEY],
        end_frames=["world_ned", "world_ned"],
        fallback_val=[fish_view_frame_hardcoded, shark_view_frame_hardcoded],
    )

    dynamic_set_surface_pose_frame = DynamicSetBlackboard(
        name=f"Select surface frame ({rubbish_name})",
        key=[choice_key, _FISH_TF_KEY, _SHARK_TF_KEY],
        update_key=_GO_SURFACE_FRAME_KEY,
        overwrite=True,
        func=lambda choice, fish_tf, shark_tf: view_frame_func(
            choice, fish_tf, shark_tf, fish_view_frame, shark_view_frame
        ),
    )

    goto_view_frame = goto.FromBlackboard(
        name=f"Go to view frame ({rubbish_name})",
        pose_key=_GO_SURFACE_FRAME_KEY,
    )

    root.add_children(
        children=[
            cluster_reset_symbols_1,
            symbol_tf_checker,
            dynamic_set_surface_pose_frame,
            goto_view_frame,
        ]
    )

    return root
