import rclpy
import rclpy.action
import rclpy.client
import rclpy.node


class TreeNode(rclpy.node.Node):

    def __init__(
        self,
        node_name: str = "tree_node",
        action_clients: dict[str, rclpy.action.ActionClient] = dict(),
        service_clients: dict[str, rclpy.client.Client] = dict(),
    ):
        super().__init__(node_name=node_name)
        self.action_clients: dict[str, rclpy.action.ActionClient] = action_clients
        self.service_clients: dict[str, rclpy.client.Client] = service_clients

    def set_action_clients(self, action_clients: dict[str, rclpy.action.ActionClient]):
        self.action_clients = action_clients

    def set_service_clients(self, service_clients: dict[str, rclpy.client.Client]):
        self.service_clients = service_clients

    def destroy_node(self):
        for action_client in self.action_clients.values():
            action_client.destroy()
        super().destroy_node()
