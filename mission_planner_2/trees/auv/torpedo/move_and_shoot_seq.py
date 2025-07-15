import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from std_srvs.srv import Trigger

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.cluster_goto import create_goto_cluster_from_bb_root
from mission_planner_2.commons.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
    within_threshold_rpy,
    within_threshold_xyz,
)
from mission_planner_2.trees.auv.goto import goto


def create_move_and_shoot_generator(
    anchor_frame_key: str,
    torpedo_shooter_left_frame: str,
    torpedo_shooter_right_frame: str,
    choice_key: str,
    pose_key: str,
    pose_frame_key: str,
    fish_shoot_frame: str,
    shark_shoot_frame: str,
    template_frame_optical: str,
    template_frame_optical_clustered: str,
    cluster_duration: int,
    realign_cluster_duration: int,
    actuation_topic_left: str,
    actuation_topic_right: str,
    distance_threshold=0.05,
    yaw_threshold=3.0,
    retries=3,
    stabilization_duration=2.5,
):
    def f(first=True):
        if first:
            anchor_frame = torpedo_shooter_left_frame
            shoot_pose_sel = lambda choice: create_stamped_pose(
                fish_shoot_frame if choice.success else shark_shoot_frame
            )
            shoot_frame_sel = lambda choice: (
                fish_shoot_frame if choice.success else shark_shoot_frame
            )
            actuation_topic = actuation_topic_left
            torp_string = "first"
        else:
            anchor_frame = torpedo_shooter_right_frame
            shoot_pose_sel = lambda choice: create_stamped_pose(
                shark_shoot_frame if choice.success else fish_shoot_frame
            )
            shoot_frame_sel = lambda choice: (
                shark_shoot_frame if choice.success else fish_shoot_frame
            )
            actuation_topic = actuation_topic_right
            torp_string = "second"

        set_anchor_frame = py_trees.behaviours.SetBlackboardVariable(
            name="Set anchor frame",
            variable_name=anchor_frame_key,
            variable_value=anchor_frame,
            overwrite=True,
        )

        dynamic_set_pose = DynamicSetBlackboard(
            name="select torpedo frame",
            key=choice_key,
            update_key=pose_key,
            overwrite=True,
            func=shoot_pose_sel,
        )

        dynamic_set_frame = DynamicSetBlackboard(
            name="select torpedo frame",
            key=choice_key,
            update_key=pose_frame_key,
            overwrite=True,
            func=shoot_frame_sel,
        )

        cluster_node = py_trees_ros.action_clients.FromConstant(
            name=f"Cluster the transforms before {torp_string} shot",
            action_type=ClusterTf,
            action_name="/auv4/cluster_tf",
            action_goal=create_clustering_goal(
                in_children=template_frame_optical,
                out_children=template_frame_optical_clustered,
                duration=cluster_duration,
                use_cache=False,
            ),
        )

        cluster_node_check = py_trees_ros.action_clients.FromConstant(
            name=f"Cluster the transforms before {torp_string} shot",
            action_type=ClusterTf,
            action_name="/auv4/cluster_tf",
            action_goal=create_clustering_goal(
                in_children=template_frame_optical,
                out_children=template_frame_optical_clustered,
                duration=realign_cluster_duration,
                use_cache=False,
            ),
        )

        goto_target = goto.FromBlackboard(
            name=f"Go to {torp_string} target",
            pose_key=pose_key,
            anchor_frame_name=anchor_frame,
        )

        goto_cluster = py_trees.decorators.FailureIsSuccess(
            name=f"Cluster and goto {torp_string}",
            child=create_goto_cluster_from_bb_root(
                cluster_node=cluster_node,
                cluster_node_check=cluster_node_check,
                goto_node=goto_target,
                start_frame_keys=[anchor_frame_key, "/global/base_link"],
                retries=retries,
                goto_pose_frame_key=pose_frame_key,
                within_threshold_list=[
                    within_threshold_xyz(distance_threshold),
                    within_threshold_rpy(yaw_threshold),
                ],
                stabilization_duration=stabilization_duration,
            ),
            # child=create_goto_cluster_from_bb_tf_tf_root(
            #     cluster_node=cluster_node_first,
            #     cluster_node_check=cluster_node_check_first,
            #     goto_node=goto_target_first,
            #     distance_threshold=0.05,
            #     retries=3,
            #     tf_frame_key=_POSE_FRAME_KEY,
            #     within_threshold=within_threshold_dist,
            #     stabilization_duration=2.5,
            # ),
        )

        fire = py_trees_ros.service_clients.FromConstant(
            name=f"Fire {torp_string} torpedo",
            service_type=Trigger,
            service_name=actuation_topic,
            service_request=Trigger.Request(),
        )

        root = py_trees.composites.Sequence(
            f"Move and shoot {torp_string} torpedo",
            memory=True,
            children=[
                set_anchor_frame,
                dynamic_set_pose,
                dynamic_set_frame,
                goto_cluster,
                fire,
            ],
        )

        return root

    return f
