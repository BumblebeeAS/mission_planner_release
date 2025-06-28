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


def create_bottle_root(
    bottle_0_frame: str = "bottle_0",
    bottle_1_frame: str = "bottle_1",
    bottle_0_view_frame: str = "bottle_0/clustered/view",
    bottle_1_view_frame: str = "bottle_1/clustered/view",
    bottle_0_frame_clustered: str = "bottle_0/clustered",
    bottle_1_frame_clustered: str = "bottle_1/clustered",
    bottle_basket_frame: str = "pink_bucket",
    bottle_basket_frame_clustered: str = "pink_bucket/clustered",
    bottle_basket_view_frame: str = "pink_bucket/clustered/view",
    cluster_duration: int = 5,
    surface_frame_key: str = "go_surface_frame",
):
    bottle_seq = py_trees.composites.Sequence(
        name="Bottle sequence",
        memory=True,
    )

    seq_pickup_bottle = py_trees.composites.Sequence(
        name="Pick up sequence (bottle)",
        memory=True,
    )

    seq_surface_bottle = py_trees.composites.Sequence(
        name="Surface sequence (bottle)",
        memory=True,
    )

    seq_drop_bottle = py_trees.composites.Sequence(
        name="Drop sequence (bottle)",
        memory=True,
    )

    cluster_bottle = py_trees_ros.actions.ActionClient(
        name="Cluster bottle",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf_multi",
        action_goal=create_clustering_goal(
            in_children=[bottle_0_frame, bottle_1_frame],
            out_children=[bottle_0_frame_clustered, bottle_1_frame_clustered],
            duration=cluster_duration,
            use_cache=False,
        ),
    )

    goto_bottle = goto.FromConstant(
        name="Go to bottle",
        pose=create_stamped_pose(frame_id=bottle_0_view_frame),
    )

    pub_half_close_grabber = py_trees_ros.publishers.FromBlackboard(
        name="half close bottle",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable="TODO: yisiong whats it now",
    )

    # now surface with the bottle facing the saved tf
    goto_surface_bottle = goto.FromBlackboard(
        name="Go to surface with bottle",
        pose_key=surface_frame_key,
        anchor_frame_name="auv4/base_link_ned",
    )

    cluster_bottle_basket = py_trees_ros.action_clients.FromConstant(
        name="Cluster bottle basket",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=bottle_basket_frame,
            out_children=bottle_basket_frame_clustered,
            duration=cluster_duration,
            use_cache=False,
        ),
    )

    goto_bottle_basket = goto.FromConstant(
        name="Go to bottle basket",
        pose=create_stamped_pose(frame_id=bottle_basket_view_frame),
    )

    # TODO: might not need this since only two objects
    pub_activate_grabber_bottle = py_trees_ros.publishers.FromBlackboard(
        name="Drop bottle",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable="TODO: yisiong whats it now",
    )

    # resurface before start of bottle
    # TODO: may need to recluster the surface pose when surface for pts (goto_surface_bottle)
    goto_surface_reset = goto.FromBlackboard(
        name="Go to surface reset",
        pose_key=surface_frame_key,
        anchor_frame_name="auv4/base_link_ned",
    )

    seq_pickup_bottle.add_children(
        [
            cluster_bottle,
            goto_bottle,
            py_trees.timers.Timer(name="Stabilise before pick up", duration=5.0),
            pub_half_close_grabber,
            py_trees.timers.Timer(name="Wait after pick up", duration=5.0),
        ]
    )

    seq_surface_bottle.add_children(
        [
            goto_surface_bottle,
            cluster_bottle_basket,
        ]
    )

    seq_drop_bottle.add_children(
        [
            goto_bottle_basket,
            pub_activate_grabber_bottle,
            py_trees.timers.Timer(name="Wait after drop", duration=5.0),
        ]
    )

    bottle_seq.add_children(
        [
            seq_pickup_bottle,
            seq_surface_bottle,
            seq_drop_bottle,
            goto_surface_reset,
        ]
    )

    return bottle_seq
