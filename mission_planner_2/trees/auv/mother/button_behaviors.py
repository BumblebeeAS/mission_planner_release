import py_trees
import py_trees_ros
from bb_controls_msgs.srv import Controller
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_localization.srv import SetPose

from mission_planner_2.commons import checked_service
from mission_planner_2.trees.auv.button.wait_for_button import (
    create_button_behavior_root,
)


def create_button_start_root(
    reset_pose_srv_topic: str,
    controls_srv_topic: str,
    left_button_topic: str,
    right_button_topic: str,
    button_retries: int,
):
    root = py_trees.composites.Sequence(
        name="Start button seq",
        memory=True,
    )

    req = SetPose.Request()
    req.pose = PoseWithCovarianceStamped()
    req.pose.pose.pose.orientation.w = 1.0

    seq_reset_enable = py_trees.composites.Sequence(
        name="Seq reset and enable controls",
        memory=True,
    )

    srv_reset_pose = py_trees_ros.service_clients.FromConstant(
        name="Reset pose",
        service_type=SetPose,
        service_name=reset_pose_srv_topic,
        service_request=req,
    )

    srv_enable_controls = checked_service.FromConstant(
        name="Enable controls",
        service_name=controls_srv_topic,
        service_type=Controller,
        service_request=Controller.Request(
            enable=True,
            pause=False,
            disable_altitude=False,
        ),
    )

    retry_enable_controls = py_trees.decorators.Retry(
        name="Retry enable controls",
        child=srv_enable_controls,
        num_failures=1000,
    )

    seq_reset_enable.add_children(
        [
            srv_reset_pose,
            retry_enable_controls,
        ]
    )

    button_reset_enable = create_button_behavior_root(
        button_topic=left_button_topic,
        num_retries=button_retries,
        execute_behavior=seq_reset_enable,
    )

    success = py_trees.behaviours.Success(name="Button success")

    button_start_mission = create_button_behavior_root(
        button_topic=right_button_topic,
        num_retries=button_retries,
        execute_behavior=success,
    )

    root.add_children(
        [
            button_reset_enable,
            button_start_mission,
        ]
    )

    return root
