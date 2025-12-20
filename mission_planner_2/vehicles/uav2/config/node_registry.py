import rclpy
import rclpy.action
import rclpy.node
from bb_perception_msgs.action import ClusterTfAction
from mission_planner_2.common.config.generic_registry import (
    ActionRegistry,
    SharedAction,
)


class UAV2SharedAction(SharedAction):
    CLUSTER = ActionRegistry("/uav2/cluster_tf", ClusterTfAction)
    GOTO = ActionRegistry("/uav2/offboard_node/go_to_position", ClusterTfAction)


class TreeNode(rclpy.node.Node):
    def __init__(self, node_name: str = "tree_node"):
        super().__init__(node_name=node_name)
        self.action_clients: dict[str, rclpy.action.ActionClient] = dict()

        for action in UAV2SharedAction:
            self.action_clients[action.name] = rclpy.action.ActionClient(
                node=self,
                action_type=action.value.type,
                action_name=action.value.topic,
            )

    def destroy_node(self):
        for action_client in self.action_clients.values():
            action_client.destroy()
        super().destroy_node()
