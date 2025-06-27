#!/usr/bin/env python3
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    launch_objects = [
        DeclareLaunchArgument(
            "static_tf_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mission_planner_2"), "cfg", "static_tfs.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "dynamic_tf_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mission_planner_2"), "cfg", "dynamic_tfs.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "default_suffix",
            default_value="view",
        ),
    ]

    nodes = [
        Node(
            package="mission_planner_2",
            executable="mission_tfs.py",
            name="mission_tf_publisher",
            parameters=[
                {
                    "static_tf_file": LaunchConfiguration("static_tf_file"),
                    "dynamic_tf_file": LaunchConfiguration("dynamic_tf_file"),
                    "default_suffix": LaunchConfiguration("default_suffix"),
                }
            ],
            output="screen",
        ),
        Node(
            package="mission_planner_2",
            executable="choice_server",
            name="rs25_choice_server",
            namespace="auv4",
            output="screen",
        ),
    ]

    return LaunchDescription(launch_objects + nodes)
