import typing
from dataclasses import dataclass
from enum import Enum

import rclpy
import rclpy.action
import rclpy.client
import rclpy.node
from bb_auv_msgs.msg import ColorRgb
from bb_behavior_msgs.action import AlignAndCollect
from bb_controls_msgs.action import Locomotion
from bb_perception_msgs.action import ClusterTfAction
from bb_planner_msgs.srv import GetPoseToControlsFrame
from rclpy.qos import qos_profile_sensor_data


@dataclass
class ActionRegistry:
    topic: str
    type: typing.Any


@dataclass
class ServiceRegistry:
    topic: str
    type: typing.Any


class SharedAction(Enum):
    LOCOMOTION = ActionRegistry("/auv4/controls", Locomotion)
    CLUSTER = ActionRegistry("/auv4/cluster_tf", ClusterTfAction)
    CLUSTER_MULTI = ActionRegistry("/auv4/cluster_tf_multi", ClusterTfAction)
    TRASH = ActionRegistry("/auv4/trash_align_and_collect", AlignAndCollect)


class SharedService(Enum):
    CONVERT_TO_CONTROLS_POSE = ServiceRegistry(
        "/auv4/convert_to_controls_pose", GetPoseToControlsFrame
    )


class TreeNode(rclpy.node.Node):
    LED_TOPIC = "/auv4/led"

    def __init__(self, node_name: str = "tree_node"):
        super().__init__(node_name=node_name)
        self.action_clients: dict[str, rclpy.action.ActionClient] = dict()
        self.service_clients: dict[str, rclpy.client.Client] = dict()

        for action in SharedAction:
            self.action_clients[action.name] = rclpy.action.ActionClient(
                node=self,
                action_type=action.value.type,
                action_name=action.value.topic,
            )
        for service in SharedService:
            self.service_clients[service.name] = self.create_client(
                srv_type=service.value.type,
                srv_name=service.value.topic,
            )

        self.led_publisher = self.create_publisher(
            ColorRgb, self.LED_TOPIC, qos_profile=qos_profile_sensor_data
        )

    def destroy_node(self):
        for action_client in self.action_clients.values():
            action_client.destroy()
        super().destroy_node()
