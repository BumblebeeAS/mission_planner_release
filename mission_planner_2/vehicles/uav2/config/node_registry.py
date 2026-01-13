import rclpy.action
import rclpy.client
import rclpy.node
from bb_perception_msgs.action import ClusterPosesAction
from bb_planner_msgs.srv import GetPoseToControlsFrame
from bb_uav_msgs.action import GoToPosition, Land, Takeoff
from mission_planner_2.common.config.generic_registry import (
    ActionRegistry,
    ServiceRegistry,
    SharedAction,
    SharedService,
)


class UAV2SharedAction(SharedAction):
    CLUSTER = ActionRegistry("/uav2/cluster_poses", ClusterPosesAction)
    GOTO = ActionRegistry("/uav2/offboard_node/go_to_position", GoToPosition)
    TAKEOFF = ActionRegistry("/uav2/offboard_node/takeoff", Takeoff)
    LAND = ActionRegistry("/uav2/offboard_node/land", Land)


class UAV2SharedService(SharedService):
    CONVERT_TO_CONTROLS_POSE = ServiceRegistry(
        "/uav2/convert_to_controls_pose", GetPoseToControlsFrame
    )


class TreeNode(rclpy.node.Node):
    def __init__(self, node_name: str = "tree_node"):
        super().__init__(node_name=node_name)
        self.action_clients: dict[str, rclpy.action.ActionClient] = dict()
        self.service_clients: dict[str, rclpy.client.Client] = dict()

        for action in UAV2SharedAction:
            self.action_clients[action.name] = rclpy.action.ActionClient(
                node=self,
                action_type=action.value.type,
                action_name=action.value.topic,
            )

        for service in UAV2SharedService:
            self.service_clients[service.name] = self.create_client(
                srv_type=service.value.type,
                srv_name=service.value.topic,
            )

    def destroy_node(self):
        for action_client in self.action_clients.values():
            action_client.destroy()
        for service_client in self.service_clients.values():
            service_client.destroy()
        super().destroy_node()
