import py_trees
import py_trees_ros
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
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.octagon.helpers import trash_view_frame_func
from mission_planner_2.trees.auv.octagon.tf_checker import create_tf_checker_root
from std_srvs.srv import Trigger

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

_LADLE_0_FRAME_KEY = fk("frame_0")
_LADLE_1_FRAME_KEY = fk("frame_1")
_BASKET_FRAME_KEY = fk("basket")
_LADLE_VIEW_FRAME_KEY = fk("ladle_view")


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
    actuation_topic: str = "/auv4/actuation/grabber",
):
    seq_ladle = py_trees.composites.Sequence(
        name="ladle sequence",
        memory=True,
    )

    seq_pickup_ladle = py_trees.composites.Sequence(
        name="Pick up sequence (ladle)",
        memory=True,
    )

    seq_surface_ladle = py_trees.composites.Sequence(
        name="Surface sequence (ladle)",
        memory=True,
    )

    seq_drop_ladle = py_trees.composites.Sequence(
        name="Drop sequence (ladle)",
        memory=True,
    )

    par_cluster = py_trees.composites.Parallel(
        name="Cluster parallel (ladle)",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )

    seq_filter_frames = py_trees.composites.Sequence(
        name="Filter frames (ladle)",
        memory=True,
    )

    cluster_ladle = py_trees_ros.actions.ActionClient(
        name="Cluster ladle",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf_multi",
        action_goal=create_clustering_goal(
            in_children=[ladle_0_frame, ladle_1_frame],
            out_children=[ladle_1_frame_clustered, ladle_0_frame_clustered],
            duration=cluster_duration,
            use_cache=False,
        ),
    )

    cluster_ladle_basket_before_goto = py_trees_ros.action_clients.FromConstant(
        name="Cluster ladle basket (for filter)",
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
            cluster_ladle,
            cluster_ladle_basket_before_goto,
        ]
    )

    # TODO: assume got basket cluster the fallback handled diferently
    ladle_tf_checker = create_tf_checker_root(
        frames=[
            ladle_0_frame_clustered,
            ladle_1_frame_clustered,
            ladle_basket_frame_clustered,
        ],
        update_keys=[_LADLE_0_FRAME_KEY, _LADLE_1_FRAME_KEY, _BASKET_FRAME_KEY],
        fallback_val=[None, None, None],
    )

    dynamic_set_ladle_pose = DynamicSetBlackboard(
        name="select ladle frame",
        key=[_LADLE_0_FRAME_KEY, _LADLE_1_FRAME_KEY, _BASKET_FRAME_KEY],
        update_key=_LADLE_VIEW_FRAME_KEY,
        overwrite=True,
        func=lambda ladle_0_tf, ladle_1_tf, ladle_basket_tf: trash_view_frame_func(
            tf_0=ladle_0_tf,
            tf_1=ladle_1_tf,
            view_frame_0=ladle_0_view_frame,
            view_frame_1=ladle_1_view_frame,
            basket_tf=ladle_basket_tf,
        ),
    )

    seq_filter_frames.add_children(
        children=[
            ladle_tf_checker,
            dynamic_set_ladle_pose,
        ]
    )

    # move down to pick up the ladle
    # TODO: need an anchor frame?
    goto_ladle_xy = goto.FromBlackboard(
        name="Go to ladle",
        pose_key=_LADLE_VIEW_FRAME_KEY,
        ignore_depth=True,
    )

    # TODO: need an anchor frame!
    goto_ladle_z = goto.FromBlackboard(
        name="Go down to ladle",
        pose_key=_LADLE_VIEW_FRAME_KEY,
    )

    # TODO: update to checked srv once elec gives feedback
    pub_half_close_grabber = py_trees_ros.service_clients.FromConstant(
        name="Close grabber (ladle)",
        service_name=actuation_topic,
        service_type=Trigger,
        service_request=Trigger.Request(),
    )

    # now surface with the ladle facing the saved tf
    goto_surface_ladle = goto.FromBlackboard(
        name="Go to surface with ladle",
        pose_key=surface_frame_key,
        anchor_frame_name="auv4/base_link_ned",
    )

    cluster_ladle_basket = py_trees_ros.action_clients.FromConstant(
        name="Cluster ladle basket (for drop)",
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

    goto_ladle_basket = goto.FromConstant(
        name="Go to ladle basket",
        pose=create_stamped_pose(frame_id=ladle_basket_view_frame),
    )

    pub_activate_grabber_ladle = py_trees_ros.service_clients.FromConstant(
        name="Open grabber (ladle)",
        service_name=actuation_topic,
        service_type=Trigger,
        service_request=Trigger.Request(),
    )

    goto_surface_reset = goto.FromBlackboard(
        name="Go to surface reset",
        pose_key=surface_frame_key,
        anchor_frame_name="auv4/base_link_ned",
    )

    seq_pickup_ladle.add_children(
        children=[
            par_cluster,
            seq_filter_frames,
            goto_ladle_xy,
            goto_ladle_z,
            py_trees.timers.Timer("Stabilise before pick up", duration=5.0),
            pub_half_close_grabber,
            py_trees.timers.Timer("Wait after pick up", duration=5.0),
        ]
    )
    seq_surface_ladle.add_children(
        children=[
            goto_surface_ladle,
            cluster_ladle_basket,
        ]
    )
    seq_drop_ladle.add_children(
        children=[
            goto_ladle_basket,
            pub_activate_grabber_ladle,
            py_trees.timers.Timer("Wait after drop", duration=5.0),
        ]
    )

    seq_ladle.add_children(
        children=[
            seq_pickup_ladle,
            seq_surface_ladle,
            seq_drop_ladle,
            goto_surface_reset,
        ]
    )
    return seq_ladle
