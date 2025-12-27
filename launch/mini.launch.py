#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_objects = [
        DeclareLaunchArgument(
            "single_tfs_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mission_planner_2"), "cfg", "single_tfs.yaml"]
            ),
        ),
        DeclareLaunchArgument(
            "grouped_tfs_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mission_planner_2"), "cfg", "grouped_tfs.yaml"]
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
            executable="mission_tfs_node.py",
            name="mission_tfs_node",
            parameters=[
                {
                    "single_tfs_file": LaunchConfiguration("single_tfs_file"),
                    "grouped_tfs_file": LaunchConfiguration("grouped_tfs_file"),
                    "default_suffix": LaunchConfiguration("default_suffix"),
                }
            ],
            output="screen",
        ),
    ]

    return LaunchDescription(launch_objects + nodes)
