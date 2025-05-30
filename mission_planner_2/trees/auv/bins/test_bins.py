import py_trees
import py_trees_ros
from bb_msgs.srv import IMPoseEstimatorToggleTemplate
from bb_perception_msgs.msg import PointCorrespondencesStamped
from rclpy.qos import qos_profile_system_default

from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.torpedo.torpedo import TOGGLE_TEMPLATE_TOPIC

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

POINT_CORRESPONDENCES_TOPIC = "/auv4/bot_cam/image_matching/point_correspondences"
TEMPLATE_NAME = "Task03_DropBRUVS.png"
TOGGLE_TEMPLATE_TOPIC = "/auv4/bot_cam/image_matching/toggle_template"


def create_test_tree_root():
    root = py_trees.composites.Sequence(name="root", memory=True)

    srv_enable_detections = py_trees_ros.service_clients.FromConstant(
        name="Enable detections",
        service_name=TOGGLE_TEMPLATE_TOPIC,
        service_type=IMPoseEstimatorToggleTemplate,
        service_request=IMPoseEstimatorToggleTemplate.Request(
            enabled=True,
            camera_frame_id="auv4/bot_cam_optical",
            template_name=TEMPLATE_NAME,
        ),
        key_response=fk("torpedo_enable_detections"),
    )

    sub_1 = py_trees_ros.subscribers.ToBlackboard(
        name="sub",
        topic_name=POINT_CORRESPONDENCES_TOPIC,
        topic_type=PointCorrespondencesStamped,
        qos_profile=qos_profile_system_default,
        blackboard_variables={fk("num_points_1"): "object_points"},
    )

    goto_second = goto.FromConstant(
        name="goto_second",
        parent_namespace=NAMESPACE,
        pose=create_stamped_pose("auv4/base_link_ned", yaw=180.0),
    )

    sub_2 = py_trees_ros.subscribers.ToBlackboard(
        name="sub",
        topic_name=POINT_CORRESPONDENCES_TOPIC,
        topic_type=PointCorrespondencesStamped,
        qos_profile=qos_profile_system_default,
        blackboard_variables={fk("num_points_2"): "object_points"},
    )

    do_check = DynamicSetBlackboard(
        name="check",
        namespace=NAMESPACE,
        key=["num_points_1", "num_points_2"],
        update_key="final",
        func=lambda x, y: create_stamped_pose("auv4/base_link_ned", yaw=180.0)
        if len(x.data) > len(y.data)
        else create_stamped_pose("auv4/base_link_ned"),
    )

    move_correct = goto.FromBlackboard(
        name="move_correct",
        parent_namespace=NAMESPACE,
        pose_key="final",
    )

    root.add_children(
        [
            srv_enable_detections,
            py_trees.timers.Timer(duration=5.0),
            sub_1,
            goto_second,
            py_trees.timers.Timer(duration=20.0),
            sub_2,
            do_check,
            move_correct,
        ]
    )

    return root
