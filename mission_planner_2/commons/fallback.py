import py_trees
import py_trees_ros
from py_trees.decorators import Retry
from rclpy.qos import qos_profile_system_default

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.tf_checker import create_tf_checker_from_constant_root


def create_clustering_fallback_root(
    cluster_node: py_trees.behaviour.Behaviour,
    world_frame: str,
    object_frame: str,
    clustered_frame: str,
    key: str,
    num_retries: int,
    fallback_val=None,
    name: str = "Cluster fallback selector",
) -> py_trees.composites.Selector:

    retry_cluster = Retry(
        name="Retry clustering", child=cluster_node, num_failures=num_retries
    )

    check_transforms = create_tf_checker_from_constant_root(
        start_frames=[
            world_frame,
        ],
        end_frames=[
            object_frame,
        ],
        update_keys=[
            key,
        ],
        fallback_val=[fallback_val],
    )

    extract_transform = DynamicSetBlackboard(
        name="Extract transform",
        key=key,
        update_key=key,  # Overwrite in place
        overwrite=True,
        func=lambda tf_stamped: tf_stamped.transform if tf_stamped else None,
    )

    publish_tf = py_trees_ros.transforms.FromBlackboard(
        name="Publish transform",
        variable_name=key,
        target_frame=clustered_frame,
        source_frame=object_frame,
        static=True,
        qos_profile=qos_profile_system_default,
        static_qos_profile=qos_profile_system_default,
    )

    seq_cluster_fallback = py_trees.composites.Selector(
        name=f"Clustering {name} fallback sequence",
        memory=True,
    )

    seq_cluster_fallback.add_children(
        [
            check_transforms,
            extract_transform,
            publish_tf,
        ]
    )

    sel_cluster_fallback = py_trees.composites.Selector(
        name=f"Clustering {name} with fallback selector",
        memory=True,
    )

    sel_cluster_fallback.add_children(
        [
            retry_cluster,
            seq_cluster_fallback,
        ]
    )

    return sel_cluster_fallback
