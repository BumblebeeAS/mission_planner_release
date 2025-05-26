import launch
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription(
        [
            Node(
                package="mission_planner_2",
                executable="choice_server",
                name="rs25_choice_server",
                namespace="auv4/",
                output="screen"
            )
        ]
    )


if __name__ == "__main__":
    launch.launch(generate_launch_description())
