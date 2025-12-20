#!/usr/bin/env python3

import math
import traceback
from typing import Any, Dict, List

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler


class MissionTfsNode(Node):
    """
    ROS2 Node responsible for mission tfs.
    """

    def __init__(self):
        super().__init__("mission_tfs_node")
        self.declare_parameter("static_tf_file", "")
        self.declare_parameter("dynamic_tf_file", "")
        self.declare_parameter("default_suffix", "view")

        static_config_path = (
            self.get_parameter("static_tf_file").get_parameter_value().string_value
        )
        dynamic_config_path = (
            self.get_parameter("dynamic_tf_file").get_parameter_value().string_value
        )
        self.default_suffix = (
            self.get_parameter("default_suffix").get_parameter_value().string_value
        )

        if not static_config_path and not dynamic_config_path:
            self.get_logger().error(
                "No config files provided! Use --ros-args -p static_tf_file:=<path> "
                "and/or -p dynamic_tf_file:=<path>"
            )
            return

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        all_transforms = []

        # Load and process static transforms
        if static_config_path:
            static_transforms = self.load_static_transforms(static_config_path)
            all_transforms.extend(static_transforms)

        # Load and process dynamic transforms
        if dynamic_config_path:
            dynamic_transforms = self.load_dynamic_transforms(dynamic_config_path)
            all_transforms.extend(dynamic_transforms)

        # Publish all transforms
        if all_transforms:
            self.tf_static_broadcaster.sendTransform(all_transforms)
            self.get_logger().info(
                f"Published {len(all_transforms)} mission transforms"
            )
            for tf in all_transforms:
                self.get_logger().info(
                    f"Published {tf.header.frame_id} -> {tf.child_frame_id}"
                )

    def load_config_file(self, config_file_path: str) -> Dict[str, Any]:
        """Load and parse YAML config file."""
        try:
            with open(config_file_path, "r") as file:
                config = yaml.safe_load(file)
            return config

        except FileNotFoundError:
            self.get_logger().error(
                f"Config file not found: {config_file_path}. "
                f"Traceback:\n{traceback.format_exc()}"
            )
            raise

        except yaml.YAMLError:
            self.get_logger().error(
                f"Error parsing YAML file: {config_file_path}. "
                f"Traceback:\n{traceback.format_exc()}"
            )
            raise

        except Exception:
            self.get_logger().error(
                f"Error loading config file: {config_file_path}. "
                f"Traceback:\n{traceback.format_exc()}"
            )
            raise

    def load_static_transforms(self, config_file_path: str) -> List[TransformStamped]:
        """Load transforms from static YAML config and return them."""
        try:
            config = self.load_config_file(config_file_path)
            static_transforms = []

            for _, transforms_list in config.items():
                for tf_config in transforms_list:
                    transform = self.create_static_transform(tf_config)
                    static_transforms.append(transform)

            self.get_logger().info(f"Loaded {len(static_transforms)} static transforms")
            return static_transforms

        except Exception:
            self.get_logger().error(
                f"Error loading static transforms. Traceback:\n{traceback.format_exc()}"
            )
            raise

    def load_dynamic_transforms(self, config_file_path: str) -> List[TransformStamped]:
        """Load transforms from dynamic YAML config and return them."""
        try:
            config = self.load_config_file(config_file_path)
            dynamic_transforms = []

            for _, dynamic_configs in config.items():
                for dynamic_config in dynamic_configs:
                    transforms = self.create_dynamic_transforms(dynamic_config)
                    dynamic_transforms.extend(transforms)

            self.get_logger().info(
                f"Loaded {len(dynamic_transforms)} dynamic transforms"
            )
            return dynamic_transforms

        except Exception:
            self.get_logger().error(
                f"Error loading dynamic transforms. Traceback:\n{traceback.format_exc()}"
            )
            raise

    def create_static_transform(self, tf_config: Dict[str, Any]) -> TransformStamped:
        """Create a TransformStamped message from static tf configuration."""
        try:
            return self.create_transform(
                x=tf_config.get("x", 0.0),
                y=tf_config.get("y", 0.0),
                z=tf_config.get("z", 0.0),
                roll=tf_config.get("roll", 0.0),
                pitch=tf_config.get("pitch", 0.0),
                yaw=tf_config.get("yaw", 0.0),
                parent_frame=tf_config["parent_frame_id"],
                child_frame=tf_config["child_frame_id"],
            )

        except Exception:
            self.get_logger().error(
                f"Error creating static transform from config: {tf_config}. "
                f"Traceback:\n{traceback.format_exc()}"
            )
            raise

    def create_dynamic_transforms(
        self, dynamic_config: Dict[str, Any]
    ) -> List[TransformStamped]:
        """Create TransformStamped messages from dynamic configuration."""
        try:
            transforms = []
            parents = dynamic_config.get("parents", [])

            suffix = dynamic_config.get("suffix", self.default_suffix)

            for parent_frame in parents:
                child_frame = f"{parent_frame}/{suffix}"
                transform = self.create_transform(
                    x=dynamic_config.get("x", 0.0),
                    y=dynamic_config.get("y", 0.0),
                    z=dynamic_config.get("z", 0.0),
                    roll=dynamic_config.get("roll", 0.0),
                    pitch=dynamic_config.get("pitch", 0.0),
                    yaw=dynamic_config.get("yaw", 0.0),
                    parent_frame=parent_frame,
                    child_frame=child_frame,
                )
                transforms.append(transform)

            return transforms

        except Exception:
            self.get_logger().error(
                f"Error creating dynamic transforms from config: {dynamic_config}. "
                f"Traceback:\n{traceback.format_exc()}"
            )
            raise

    def create_transform(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        parent_frame: str,
        child_frame: str,
    ) -> TransformStamped:
        """Create a TransformStamped message from transform parameters."""
        try:
            roll_rad = math.radians(roll)
            pitch_rad = math.radians(pitch)
            yaw_rad = math.radians(yaw)

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
                f"Error creating transform. " f"Traceback:\n{traceback.format_exc()}"
            )
            raise


def main(args=None):
    rclpy.init(args=args)
    mission_tfs_node = MissionTfsNode()
    try:
        rclpy.spin(mission_tfs_node)
    except KeyboardInterrupt:
        pass
    finally:
        mission_tfs_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
