import py_trees
import py_trees_ros
from std_srvs.srv import Trigger

from mission_planner_2.commons import shared_action_client
from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.cluster_goto import create_goto_cluster_from_bb_root
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.node_registry import SharedAction
from mission_planner_2.commons.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
    within_threshold_rpy,
    within_threshold_xyz,
)
from mission_planner_2.trees.auv.goto import goto

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

_CLUSTERING_GOAL_KEY = fk("clustering_goal")
_CLUSTERING_GOAL_CHECK_KEY = fk("clustering_goal_check")


def create_move_and_shoot_generator(
    anchor_frame_key: str,
    torpedo_shooter_left_frame: str,
    torpedo_shooter_right_frame: str,
    choice_key: str,
    pose_key: str,
    pose_frame_key: str,
    fish_shoot_frame_key: str,
    shark_shoot_frame_key: str,
    template_frame_optical_key: str,
    template_frame_optical_clustered_key: str,
    cluster_duration: int,
    realign_cluster_duration: int,
    actuation_topic_left: str,
    actuation_topic_right: str,
    distance_threshold=0.05,
    yaw_threshold=3.0,
    retries=3,
    stabilization_duration=2.5,
    num_retries_clustering=3,
    wait_after_fire_duration: float = 3.0,
):
    def f(first=True):
        if first:
            anchor_frame = torpedo_shooter_left_frame
            shoot_pose_sel = (
                lambda choice, fish_shoot_frame, shark_shoot_frame: create_stamped_pose(
                    fish_shoot_frame if choice.success else shark_shoot_frame
                )
            )
            shoot_frame_sel = lambda choice, fish_shoot_frame, shark_shoot_frame: (
                fish_shoot_frame if choice.success else shark_shoot_frame
            )
            actuation_topic = actuation_topic_left
            torp_string = "first"
        else:
            anchor_frame = torpedo_shooter_right_frame
            shoot_pose_sel = (
                lambda choice, fish_shoot_frame, shark_shoot_frame: create_stamped_pose(
                    shark_shoot_frame if choice.success else fish_shoot_frame
                )
            )
            shoot_frame_sel = lambda choice, fish_shoot_frame, shark_shoot_frame: (
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
            name="select torpedo pose",
            key=[choice_key, fish_shoot_frame_key, shark_shoot_frame_key],
            update_key=pose_key,
            overwrite=True,
            func=shoot_pose_sel,
        )

        dynamic_set_frame = DynamicSetBlackboard(
            name="select torpedo frame",
            key=[choice_key, fish_shoot_frame_key, shark_shoot_frame_key],
            update_key=pose_frame_key,
            overwrite=True,
            func=shoot_frame_sel,
        )

        dynamic_set_cluster_goal = DynamicSetBlackboard(
            name="set clustering goal",
            key=[template_frame_optical_key, template_frame_optical_clustered_key],
            update_key=_CLUSTERING_GOAL_KEY,
            overwrite=True,
            func=lambda frame, clustered: create_clustering_goal(
                in_children=frame,
                out_children=clustered,
                duration=cluster_duration,
                use_cache=False,
            ),
        )

        dynamic_set_cluster_goal_check = DynamicSetBlackboard(
            name="set clustering goal check",
            key=[template_frame_optical_key, template_frame_optical_clustered_key],
            update_key=_CLUSTERING_GOAL_CHECK_KEY,
            overwrite=True,
            func=lambda frame, clustered: create_clustering_goal(
                in_children=frame,
                out_children=clustered,
                duration=realign_cluster_duration,
                use_cache=False,
            ),
        )

        cluster_node = shared_action_client.FromBlackboard(
            name=f"Cluster the transforms before {torp_string} shot",
            shared_action=SharedAction.CLUSTER,
            key=_CLUSTERING_GOAL_KEY,
        )

        retry_cluster_node = py_trees.decorators.Retry(
            name="Retry cluster node",
            child=cluster_node,
            num_failures=num_retries_clustering,
        )

        cluster_node_check = shared_action_client.FromBlackboard(
            name=f"Cluster the transforms before {torp_string} shot",
            shared_action=SharedAction.CLUSTER,
            key=_CLUSTERING_GOAL_CHECK_KEY,
        )

        retry_cluster_node_check = py_trees.decorators.Retry(
            name="Retry cluster node check",
            child=cluster_node_check,
            num_failures=num_retries_clustering,
        )

        goto_target = goto.FromBlackboard(
            name=f"Go to {torp_string} target",
            pose_key=pose_key,
            anchor_frame_name=anchor_frame,
        )

        goto_cluster = py_trees.decorators.FailureIsSuccess(
            name=f"Cluster and goto {torp_string}",
            child=create_goto_cluster_from_bb_root(
                cluster_node=retry_cluster_node,
                cluster_node_check=retry_cluster_node_check,
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

        fire_2 = py_trees_ros.service_clients.FromConstant(
            name=f"Fire {torp_string} torpedo retry",
            service_type=Trigger,
            service_name=actuation_topic,
            service_request=Trigger.Request(),
        )

        wait_after_fire = py_trees.timers.Timer(
            name="Wait after fire",
            duration=wait_after_fire_duration,
        )

        root = py_trees.composites.Sequence(
            f"Move and shoot {torp_string} torpedo",
            memory=True,
            children=[
                set_anchor_frame,
                dynamic_set_pose,
                dynamic_set_frame,
                dynamic_set_cluster_goal,
                dynamic_set_cluster_goal_check,
                goto_cluster,
                fire,
                fire_2,
                wait_after_fire,
            ],
        )

        return root

    return f
