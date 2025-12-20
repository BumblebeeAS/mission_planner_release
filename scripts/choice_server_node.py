#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


class ChoiceServerNode(Node):
    """
    ROS2 Node responsible for RS25 fish/shark choice.
    """

    def __init__(self):
        super().__init__("choice_server_node")

        self.is_fish = True

        self.set_is_fish_srv = self.create_service(
            SetBool, "/auv4/choice/set_is_fish", self.handle_set_is_fish
        )

        self.get_is_fish_srv = self.create_service(
            Trigger, "/auv4/choice/get_is_fish", self.handle_get_is_fish
        )

        self.get_logger().info("choice server ready, init to fish")

    def handle_set_is_fish(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self.is_fish = request.data
        response.success = True
        response.message = "fish" if self.is_fish else "shark"
        self.get_logger().info(f"Choice set to {'fish' if self.is_fish else 'shark'}")
        return response

    def handle_get_is_fish(self, _, response: Trigger.Response) -> Trigger.Response:
        response.success = self.is_fish
        response.message = "fish" if self.is_fish else "shark"
        self.get_logger().info(
            f"Responding to queries henceforth with {'fish' if self.is_fish else 'shark'}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    choice_server_node = ChoiceServerNode()
    try:
        rclpy.spin(choice_server_node)
    except KeyboardInterrupt:
        pass
    finally:
        choice_server_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
