#!/usr/bin/env python3
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("mission_planner_2"), "cfg", "mission_tfs.yaml"]
        ),
    )

    mission_tf_node = Node(
        package="mission_planner_2",
        executable="mission_tfs.py",
        name="mission_tf_publisher",
        parameters=[{"config_file": LaunchConfiguration("params_file")}],
        output="screen",
    )

    service_node = Node(
        package="mission_planner_2",
        executable="choice_server",
        name="rs25_choice_server",
        namespace="auv4",
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(declare_params_file)

    ld.add_action(mission_tf_node)
    ld.add_action(service_node)

    return ld
