import py_trees
import py_trees_ros
from bb_auv_msgs.action import Grabber
from bb_perception_msgs.action import ClusterTf

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
)
from mission_planner_2.commons.tf_checker import create_tf_checker_from_constant_root
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.octagon.helpers import trash_view_frame_func

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

_BOTTLE_0_FRAME_KEY = fk("frame_0")
_BOTTLE_1_FRAME_KEY = fk("frame_1")
_BASKET_FRAME_KEY = fk("basket")
_BOTTLE_VIEW_FRAME_KEY = fk("bottle_view")


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
    actuation_topic: str = "/auv4/actuation/grabber",
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

    par_cluster = py_trees.composites.Parallel(
        name="Cluster parallel (bottle)",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )

    seq_filter_frames = py_trees.composites.Sequence(
        name="Filter frames (bottle)",
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

    cluster_bottle_basket_before_goto = py_trees_ros.action_clients.FromConstant(
        name="Cluster bottle basket (for filter)",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=bottle_basket_frame,
            out_children=bottle_basket_frame_clustered,
            duration=cluster_duration,
            use_cache=False,
            persistent=False,
        ),
    )

    par_cluster.add_children(
        children=[
            cluster_bottle,
            cluster_bottle_basket_before_goto,
        ]
    )

    bottle_tf_checker = create_tf_checker_from_constant_root(
        start_frames=[
            bottle_0_frame_clustered,
            bottle_1_frame_clustered,
            bottle_basket_frame_clustered,
        ],
        end_frames=["world_ned", "world_ned", "world_ned"],
        update_keys=[_BOTTLE_0_FRAME_KEY, _BOTTLE_1_FRAME_KEY, _BASKET_FRAME_KEY],
        fallback_val=[None, None, None],
    )

    dynamic_set_bottle_pose = DynamicSetBlackboard(
        name="select bottle frame",
        key=[_BOTTLE_0_FRAME_KEY, _BOTTLE_1_FRAME_KEY, _BASKET_FRAME_KEY],
        update_key=_BOTTLE_VIEW_FRAME_KEY,
        overwrite=True,
        func=lambda bottle_0_tf, bottle_1_tf, bottle_basket_tf: trash_view_frame_func(
            tf_0=bottle_0_tf,
            tf_1=bottle_1_tf,
            view_frame_0=bottle_0_view_frame,
            view_frame_1=bottle_1_view_frame,
            basket_tf=bottle_basket_tf,
        ),
    )

    seq_filter_frames.add_children(
        children=[
            bottle_tf_checker,
            dynamic_set_bottle_pose,
        ]
    )

    goto_bottle = goto.FromConstant(
        name="Go to bottle",
        pose=create_stamped_pose(frame_id=bottle_0_view_frame),
    )

    pub_half_close_grabber = py_trees_ros.action_clients.FromConstant(
        name="Close grabber (bottle)",
        action_name=actuation_topic,
        action_type=Grabber,
        action_goal=Grabber.Goal(
            command=Grabber.Goal.GOAL_CLOSE,
            tolerance=0,
            timeout_ms=10000,  # 10 seconds
        ),
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

    pub_activate_grabber_bottle = py_trees_ros.action_clients.FromConstant(
        name="Open grabber (bottle)",
        action_name=actuation_topic,
        action_type=Grabber,
        action_goal=Grabber.Goal(
            command=Grabber.Goal.GOAL_OPEN,
            tolerance=0,
            timeout_ms=10000,  # 10 seconds
        ),
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
            par_cluster,
            seq_filter_frames,
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
