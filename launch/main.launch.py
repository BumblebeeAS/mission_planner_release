#!/usr/bin/env python3

import math

import yaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _get_tf_params(config, key):
    tf_params = config.get(key, {}).get("ros__parameters", {})
    return (
        tf_params.get("x", 0.0),
        tf_params.get("y", 0.0),
        tf_params.get("z", 0.0),
        tf_params.get("roll", 0.0),
        tf_params.get("pitch", 0.0),
        tf_params.get("yaw", 0.0),
        tf_params.get("parent_frame_id", "base_link"),
        tf_params.get("child_frame_id", "sensor_frame"),
    )


def launch_setup(context, *args, **kwargs):
    config_file_path = LaunchConfiguration("params_file").perform(context)

    with open(config_file_path, "r") as file:
        config = yaml.safe_load(file)

    # service_params = config.get("choice_srv", {}).get("ros__parameters", {})

    (
        x_shark,
        y_shark,
        z_shark,
        roll_deg_shark,
        pitch_deg_shark,
        yaw_deg_shark,
        parent_frame_shark,
        child_frame_shark,
    ) = _get_tf_params(config, "static_transform_shark")

    (
        x_fish,
        y_fish,
        z_fish,
        roll_deg_fish,
        pitch_deg_fish,
        yaw_deg_fish,
        parent_frame_fish,
        child_frame_fish,
    ) = _get_tf_params(config, "static_transform_fish")

    # Get service node parameters
    # TODO:
    # service_package = service_params.get("package", "your_cpp_package_name")
    # service_executable = service_params.get("executable", "your_service_node")
    # service_name = service_params.get("name", "service_node")

    roll_rad_shark = math.radians(roll_deg_shark)
    pitch_rad_shark = math.radians(pitch_deg_shark)
    yaw_rad_shark = math.radians(yaw_deg_shark)
    roll_rad_fish = math.radians(roll_deg_fish)
    pitch_rad_fish = math.radians(pitch_deg_fish)
    yaw_rad_fish = math.radians(yaw_deg_fish)

    # Create the static transform publisher node
    shark_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_shark",
        arguments=[
            str(x_shark),
            str(y_shark),
            str(z_shark),
            str(yaw_rad_shark),
            str(pitch_rad_shark),
            str(roll_rad_shark),
            parent_frame_shark,
            child_frame_shark,
        ],
    )
    fish_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_fish",
        arguments=[
            str(x_fish),
            str(y_fish),
            str(z_fish),
            str(yaw_rad_fish),
            str(pitch_rad_fish),
            str(roll_rad_fish),
            parent_frame_fish,
            child_frame_fish,
        ],
    )

    # TODO: Create the service node
    # service_node = Node(
    #     package=service_package,
    #     executable=service_executable,
    #     name=service_name,
    #     parameters=[
    #         config_file_path
    #     ],  # Pass the entire config file to the service node
    #     output="screen",
    # )

    return [shark_tf_node, fish_tf_node]


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
