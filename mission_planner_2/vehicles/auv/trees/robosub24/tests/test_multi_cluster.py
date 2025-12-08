import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf

from mission_planner_2.common.util.pose_utils import create_clustering_goal


def create_test_multi_cluster_root():
    root = py_trees.composites.Sequence("root", memory=True)

    in_children = ["gate", "bin/yolo"]
    out_children = ["gate/clustered", "bin/yolo/clustered"]

    action = py_trees_ros.action_clients.FromConstant(
        name="cluster",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=in_children,
            out_children=out_children,
            duration=40,
            persistent=True,
        ),
    )

    root.add_children([action])

    return root
