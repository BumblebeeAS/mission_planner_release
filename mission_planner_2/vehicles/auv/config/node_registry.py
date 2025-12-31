import rclpy
import rclpy.action
from bb_auv_msgs.action import Grabber
from bb_auv_msgs.msg import ColorRgb
from bb_behavior_msgs.action import AlignAndCollect, ControlledAscent, ControlledSpin
from bb_controls_msgs.action import Locomotion
from bb_perception_msgs.action import ClusterTfAction
from bb_planner_msgs.srv import GetPoseToControlsFrame
from rclpy.qos import qos_profile_sensor_data

from mission_planner_2.common.config.generic_registry import (
    ActionRegistry,
    ServiceRegistry,
    SharedAction,
    SharedService,
)
from mission_planner_2.common.core.tree_node import TreeNode


class AUVSharedAction(SharedAction):
    LOCOMOTION = ActionRegistry("/auv4/controls", Locomotion)
    CLUSTER = ActionRegistry("/auv4/cluster_tf", ClusterTfAction)
    CLUSTER_MULTI = ActionRegistry("/auv4/cluster_tf_multi", ClusterTfAction)
    TRASH = ActionRegistry("/auv4/trash_align_and_collect", AlignAndCollect)
    CONTROLLED_ASCENT = ActionRegistry("/auv4/controlled_ascent", ControlledAscent)
    CONTROLLED_SPIN = ActionRegistry("/auv4/controlled_spin", ControlledSpin)
    GRABBER = ActionRegistry("/auv4/actuation/grabber", Grabber)
    ALIGN_AND_COLLECT = ActionRegistry("/auv4/align_and_collect", AlignAndCollect)


class AUVSharedService(SharedService):
    CONVERT_TO_CONTROLS_POSE = ServiceRegistry(
        "/auv4/convert_to_controls_pose", GetPoseToControlsFrame
    )


class AUVTreeNode(TreeNode):
    def __init__(self, node_name: str = "auv_tree_node", led_topic: str = "/auv4/led"):
        super().__init__(
            node_name=node_name,
        )
        self.led_publisher = self.create_publisher(
            ColorRgb, led_topic, qos_profile=qos_profile_sensor_data
        )
        self._register_action_clients()
        self._register_service_clients()

    def _register_action_clients(self):
        action_clients = dict()
        for action in AUVSharedAction:
            action_clients[action.name] = rclpy.action.ActionClient(
                node=self,
                action_type=action.value.type,
                action_name=action.value.topic,
            )
        self.set_action_clients(action_clients)

    def _register_service_clients(self):
        service_clients = dict()
        for service in AUVSharedService:
            service_clients[service.name] = self.create_client(
                srv_type=service.value.type,
                srv_name=service.value.topic,
            )
        self.set_service_clients(service_clients)
