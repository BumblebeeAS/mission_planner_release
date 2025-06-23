#!/usr/bin/env python3
import math
import traceback

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler


class MissionTfPublisher(Node):
    def __init__(self):
        super().__init__("mission_tf_publisher")

        self.declare_parameter("config_file", "")
        config_file_path = (
            self.get_parameter("config_file").get_parameter_value().string_value
        )

        if not config_file_path:
            self.get_logger().error(
                "Config file path not provided! Use --ros-args -p config_file:=<path>"
            )
            return

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.load_and_publish_transforms(config_file_path)

    def load_and_publish_transforms(self, config_file_path):
        """Load transforms from YAML config and publish them."""
        try:
            with open(config_file_path, "r") as file:
                config = yaml.safe_load(file)

            tfs_config = config.get("tfs", [])
            static_transforms = []

            for tf_config in tfs_config:
                transform = self.create_transform_from_config(tf_config)
                static_transforms.append(transform)

            self.tf_static_broadcaster.sendTransform(static_transforms)
            self.get_logger().info(
                f"Published {len(static_transforms)} static transforms"
            )

            for tf in static_transforms:
                self.get_logger().info(
                    f"Published {tf.header.frame_id} -> {tf.child_frame_id}"
                )

        except FileNotFoundError:
            self.get_logger().error(
                f"Config file not found: {config_file_path}. Traceback:\n{traceback.format_exc()}"
            )
            raise
        except yaml.YAMLError:
            self.get_logger().error(
                f"Error parsing YAML file. Traceback:\n{traceback.format_exc()}"
            )
            raise
        except Exception:
            self.get_logger().error(
                f"Error loading transforms. Traceback:\n{traceback.format_exc()}"
            )
            raise

    def create_transform_from_config(self, tf_config):
        """Create a TransformStamped message from tf configuration."""
        try:
            roll_rad = math.radians(tf_config["roll"])
            pitch_rad = math.radians(tf_config["pitch"])
            yaw_rad = math.radians(tf_config["yaw"])

            x = tf_config["x"]
            y = tf_config["y"]
            z = tf_config["z"]
            parent_frame = tf_config["parent_frame_id"]
            child_frame = tf_config["child_frame_id"]

            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = parent_frame
            transform.child_frame_id = child_frame

            transform.transform.translation.x = float(x)
            transform.transform.translation.y = float(y)
            transform.transform.translation.z = float(z)

            quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)

            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]

            return transform

        except Exception:
            self.get_logger().error(
                f"Error creating transform from config. Traceback:\n{traceback.format_exc()}"
            )
            raise


def main(args=None):
    rclpy.init(args=args)

    try:
        mission_tf_publisher = MissionTfPublisher()
        rclpy.spin(mission_tf_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
