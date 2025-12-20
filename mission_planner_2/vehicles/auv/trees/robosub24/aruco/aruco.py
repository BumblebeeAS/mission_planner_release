import py_trees
from bb_perception_msgs.srv import ArucoToggleActivation
from geometry_msgs.msg import TransformStamped
from py_trees_ros import service_clients
from robot_localization.srv import SetPose

from mission_planner_2.common.core import checked_service, shared_action_client
from mission_planner_2.common.util.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.common.util.pose_utils import create_clustering_goal
from mission_planner_2.vehicles.auv.config.node_registry import AUVSharedAction
from mission_planner_2.vehicles.shared.trees.blackboard import DynamicSetBlackboard
from mission_planner_2.vehicles.shared.trees.tf_checker import (
    create_tf_checker_from_constant_root,
)

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################

ARUCO_TOGGLE_TOPIC = "/auv4/aruco_detector/toggle"
NUM_RETRIES = 3
CLUSTERING_DURATION = 7
TF_CHECK_TIMEOUT = 5
RESET_POSE_SRV = "/auv4/nav/reset_pose"

#########################################################################
_ARUCO_TF_KEY = fk("aruco_tf_key")
_RESET_POSE_REQ_KEY = fk("reset_pose_req_key")
_RESET_POSE_RESP_KEY = fk("reset_pose_resp_key")


def create_aruco_root():
    root = py_trees.composites.Sequence(
        name="Aruco localisation",
        memory=True,
    )

    srv_enable_detections = checked_service.FromConstant(
        name="Enable detections",
        service_type=ArucoToggleActivation,
        service_name=ARUCO_TOGGLE_TOPIC,
        service_request=ArucoToggleActivation.Request(enable=True),
        check_func=lambda x: x.status,
    )

    retry_enable_detections = py_trees.decorators.Retry(
        name="Retry enable detections",
        child=srv_enable_detections,
        num_failures=NUM_RETRIES,
    )

    action_cluster_detections = shared_action_client.FromConstant(
        name="Cluster detections",
        shared_action=AUVSharedAction.CLUSTER,
        action_goal=create_clustering_goal(
            in_children="aruco_board",
            out_children="aruco_board/clustered",
            duration=CLUSTERING_DURATION,
        ),
    )

    get_aruco_tf = create_tf_checker_from_constant_root(
        start_frames=["aruco_board/clustered"],
        end_frames=["auv4/base_link_ned"],
        timeout=TF_CHECK_TIMEOUT,
        update_keys=[_ARUCO_TF_KEY],
        fallback_val=[None],
    )

    create_service_request = DynamicSetBlackboard(
        name="Create service request",
        key=_ARUCO_TF_KEY,
        update_key=_RESET_POSE_REQ_KEY,
        overwrite=True,
        func=convert_tf_to_req,
    )

    srv_reset_pose = service_clients.FromBlackboard(
        name="Reset pose",
        service_type=SetPose,
        service_name=RESET_POSE_SRV,
        key_request=_RESET_POSE_REQ_KEY,
        key_response=_RESET_POSE_RESP_KEY,
    )

    srv_disable_detections = checked_service.FromConstant(
        name="Disable detections",
        service_type=ArucoToggleActivation,
        service_name=ARUCO_TOGGLE_TOPIC,
        service_request=ArucoToggleActivation.Request(enable=False),
        check_func=lambda x: not x.status,
    )

    retry_disable_detections = py_trees.decorators.Retry(
        name="Retry disable detections",
        child=srv_disable_detections,
        num_failures=NUM_RETRIES,
    )

    root.add_children(
        [
            retry_enable_detections,
            action_cluster_detections,
            get_aruco_tf,
            create_service_request,
            srv_reset_pose,
            retry_disable_detections,
        ]
    )

    return root


def convert_tf_to_req(tf: TransformStamped):
    req = SetPose.Request()

    req.pose.pose.pose.position.x = tf.transform.translation.x
    req.pose.pose.pose.position.y = tf.transform.translation.y
    req.pose.pose.pose.position.z = tf.transform.translation.z

    req.pose.pose.pose.orientation.x = tf.transform.rotation.x
    req.pose.pose.pose.orientation.y = tf.transform.rotation.y
    req.pose.pose.pose.orientation.z = tf.transform.rotation.z
    req.pose.pose.pose.orientation.w = tf.transform.rotation.w

    return req
