import uuid

import py_trees
from bb_planner_msgs.srv import GetPoseToControlsFrame
from geometry_msgs.msg import PoseStamped
from mission_planner_2.common.core import shared_service_client
from mission_planner_2.common.util.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.vehicles.shared.trees.blackboard import DynamicSetBlackboard
from mission_planner_2.vehicles.uav2.config.node_registry import UAV2SharedService
from mission_planner_2.vehicles.uav2.trees.goto import goto

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)


def _create_srv_req(
    pose: PoseStamped, anchor_frame_name: str
) -> GetPoseToControlsFrame.Request:
    req = GetPoseToControlsFrame.Request()
    req.anchor_frame_name = anchor_frame_name
    req.input_poses = [pose]
    return req


def create_goto_pose_root(
    pose: PoseStamped,
    anchor_frame_name: str = "uav2/base_link_frd",
    x_threshold: float = 0.03,
    y_threshold: float = 0.03,
    z_threshold: float = 0.03,
) -> py_trees.behaviour.Behaviour:
    response_key = fk(f"response_{str(uuid.uuid4()).replace('-', '')}")
    pose_key = fk(f"pose_{str(uuid.uuid4()).replace('-', '')}")

    seq_goto_pose = py_trees.composites.Sequence(
        name="Goto pose root",
        memory=True,
    )

    get_controls_pose = shared_service_client.FromConstant(
        name="Get controls pose",
        shared_service=UAV2SharedService.CONVERT_TO_CONTROLS_POSE,
        service_request=_create_srv_req(pose, anchor_frame_name),
        key_response=response_key,
    )
    set_pose_bb = DynamicSetBlackboard(
        name="Set pose to bb",
        key=[response_key],
        update_key=pose_key,
        func=lambda resp: resp.output_poses[0],
        overwrite=True,
    )
    goto_controls_pose = goto.FromBlackboard(
        name="Goto controls pose",
        pose_key=pose_key,
        x_threshold=x_threshold,
        y_threshold=y_threshold,
        z_threshold=z_threshold,
    )

    seq_goto_pose.add_children(
        children=[
            get_controls_pose,
            set_pose_bb,
            goto_controls_pose,
        ]
    )

    return seq_goto_pose
