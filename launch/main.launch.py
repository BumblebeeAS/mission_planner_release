#!/usr/bin/env python3

import math

import yaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _create_tf_node_from_config(tf_config):
    """Create a static transform publisher node from tf configuration."""
    # Convert degrees to radians
    roll_rad = math.radians(tf_config.get("roll", 0.0))
    pitch_rad = math.radians(tf_config.get("pitch", 0.0))
    yaw_rad = math.radians(tf_config.get("yaw", 0.0))

    # Extract position and frame information
    x = tf_config.get("x", 0.0)
    y = tf_config.get("y", 0.0)
    z = tf_config.get("z", 0.0)
    parent_frame = tf_config.get("parent_frame_id", "base_link")
    child_frame = tf_config.get("child_frame_id", "sensor_frame")
    tf_name = tf_config.get("name", "static_transform")

    # Create the static transform publisher node
    tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_transform_{tf_name}",
        arguments=[
            str(x),
            str(y),
            str(z),
            str(yaw_rad),
            str(pitch_rad),
            str(roll_rad),
            parent_frame,
            child_frame,
        ],
    )

    return tf_node


def launch_setup(context, *args, **kwargs):
    config_file_path = LaunchConfiguration("params_file").perform(context)

    with open(config_file_path, "r") as file:
        config = yaml.safe_load(file)

    # Get all transform configurations from the 'tfs' list
    tfs_config = config.get("tfs", [])

    # Create transform nodes for all configured transforms
    tf_nodes = []
    for tf_config in tfs_config:
        tf_node = _create_tf_node_from_config(tf_config)
        tf_nodes.append(tf_node)

    # Create the service node
    service_node = Node(
        package="mission_planner_2",
        executable="choice_server",
        name="rs25_choice_server",
        namespace="auv4",
        output="screen",
    )

    tf_nodes.append(service_node)

    return tf_nodes


def generate_launch_description():
    # Declare launch arguments
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("mission_planner_2"), "cfg", "cfg.yaml"]
        ),
        description="Full path to the ROS2 parameters file to use",
    )

    # Use opaque function to process parameters and create nodes
    opaque_function = OpaqueFunction(function=launch_setup)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the declarations
    ld.add_action(declare_params_file)

    # Add the opaque function
    ld.add_action(opaque_function)

    return ld
