import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger

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

_FRAME_1_KEY = fk("frame_1")
_FRAME_0_KEY = fk("frame_0")


def create_ladle_root(
    ladle_0_frame: str = "ladle_0",
    ladle_1_frame: str = "ladle_1",
    ladle_0_view_frame: str = "ladle_0/clustered/view",
    ladle_1_view_frame: str = "ladle_1/clustered/view",
    ladle_0_frame_clustered: str = "ladle_0/clustered",
    ladle_1_frame_clustered: str = "ladle_1/clustered",
    ladle_basket_frame: str = "ladle_basket",
    ladle_basket_frame_clustered: str = "ladle_basket/clustered",
    ladle_basket_view_frame: str = "ladle_basket/clustered/view",
    cluster_duration: int = 5,
    surface_frame_key: str = "go_surface_frame",
):
    seq_spoon = py_trees.composites.Sequence(
        name="Spoon sequence",
        memory=True,
    )

    seq_pickup_spoon = py_trees.composites.Sequence(
        name="Pick up sequence (spoon)",
        memory=True,
    )

    seq_surface_spoon = py_trees.composites.Sequence(
        name="Surface sequence (spoon)",
        memory=True,
    )

    seq_drop_spoon = py_trees.composites.Sequence(
        name="Drop sequence (spoon)",
        memory=True,
    )

    par_cluster = py_trees.composites.Parallel(
        name="Cluster parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )

    cluster_spoon = py_trees_ros.actions.ActionClient(
        name="Cluster spoon",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf_multi",
        action_goal=create_clustering_goal(
            in_children=[ladle_0_frame, ladle_1_frame],
            out_children=[ladle_1_frame_clustered, ladle_0_frame_clustered],
            duration=cluster_duration,
            use_cache=False,
        ),
    )

    cluster_spoon_basket_before_goto = py_trees_ros.actions.ActionClient(
        name="Cluster spoon basket (for filter)",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=ladle_basket_frame,
            out_children=ladle_basket_frame_clustered,
            duration=cluster_duration,
            use_cache=False,
            persistent=True,  # TODO: should use persistent?
        ),
    )

    par_cluster.add_children(
        children=[
            cluster_spoon,
            cluster_spoon_basket_before_goto,
        ]
    )

    # move down to pick up the spoon
    goto_spoon_xy = goto.FromConstant(
        name="Go to spoon",
        pose=create_stamped_pose(frame_id=ladle_0_view_frame),
        ignore_depth=True,
    )
    # TODO: need check if this works
    goto_spoon_z = goto.FromConstant(
        name="Go down to spoon",
        pose=create_stamped_pose(frame_id=ladle_0_view_frame),
    )

    pub_half_close_grabber = py_trees_ros.publishers.FromBlackboard(
        name="half close spoon",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable="TODO: yisiong whats it now",
    )

    # now surface with the spoon facing the saved tf
    goto_surface_spoon = goto.FromBlackboard(
        name="Go to surface with spoon",
        pose_key=surface_frame_key,
        anchor_frame_name="auv4/base_link_ned",
    )

    cluster_spoon_basket = py_trees_ros.action_clients.FromConstant(
        name="Cluster spoon basket (for drop)",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf_multi",
        action_goal=create_clustering_goal(
            in_children=ladle_basket_frame,
            out_children=ladle_basket_frame_clustered,
            duration=cluster_duration,
            use_cache=False,
            persistent=True,  # TODO: should use persistent?
        ),
    )

    goto_spoon_basket = goto.FromConstant(
        name="Go to spoon basket",
        pose=create_stamped_pose(frame_id=ladle_basket_view_frame),
    )

    pub_activate_grabber_spoon = py_trees_ros.publishers.FromBlackboard(
        name="Drop spoon",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable="TODO: yisiong whats it now",
    )

    goto_surface_reset = goto.FromBlackboard(
        name="Go to surface reset",
        pose_key=surface_frame_key,
        anchor_frame_name="auv4/base_link_ned",
    )

    seq_pickup_spoon.add_children(
        children=[
            par_cluster,
            goto_spoon_xy,
            goto_spoon_z,
            py_trees.timers.Timer("Stabilise before pick up", duration=5.0),
            pub_half_close_grabber,
            py_trees.timers.Timer("Wait after pick up", duration=5.0),
        ]
    )
    seq_surface_spoon.add_children(
        children=[
            goto_surface_spoon,
            cluster_spoon_basket,
        ]
    )
    seq_drop_spoon.add_children(
        children=[
            goto_spoon_basket,
            pub_activate_grabber_spoon,
            py_trees.timers.Timer("Wait after drop", duration=5.0),
        ]
    )

    seq_spoon.add_children(
        children=[
            seq_pickup_spoon,
            seq_surface_spoon,
            seq_drop_spoon,
            goto_surface_reset,
        ]
    )
    return seq_spoon
