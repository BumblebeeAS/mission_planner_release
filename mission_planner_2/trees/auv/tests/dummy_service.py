#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class DummyService(Node):
    def __init__(self):
        super().__init__("dummy_service")
        self.srv = self.create_service(
            SetBool, "/auv4/test_service", self.set_bool_callback
        )
        self.get_logger().info("Dummy SetBool service started")

    def set_bool_callback(self, request, response):
        # Log the incoming request
        self.get_logger().info(f"Received SetBool request: data={request.data}")

        # Set response fields
        response.success = True
        response.message = (
            f"Successfully processed SetBool request with data: {request.data}"
        )

        # You can customize the response based on the request if needed
        if request.data:
            self.get_logger().info("Request data is True")
        else:
            self.get_logger().info("Request data is False")

        return response


def main():
    rclpy.init()
    dummy_service = DummyService()

    try:
        rclpy.spin(dummy_service)
    except KeyboardInterrupt:
        dummy_service.get_logger().info("Dummy service stopped by user")
    except Exception as e:
        dummy_service.get_logger().error(f"An error occurred: {e}")
    finally:
        dummy_service.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
