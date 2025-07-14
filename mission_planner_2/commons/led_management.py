#!/usr/bin/env python3
import functools
from dataclasses import dataclass

import py_trees
import py_trees_ros
from bb_auv_msgs.msg import ColorRgb
from py_trees.visitors import VisitorBase
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data

from mission_planner_2.trees.auv.goto import goto

RED = ColorRgb(
    red=255,
    green=0,
    blue=0,
)
GREEN = ColorRgb(
    red=0,
    green=255,
    blue=0,
)
BLUE = ColorRgb(
    red=0,
    green=0,
    blue=255,
)
YELLOW = ColorRgb(
    red=255,
    green=255,
    blue=0,
)
PURPLE = ColorRgb(
    red=128,
    green=0,
    blue=128,
)
ORANGE = ColorRgb(
    red=255,
    green=165,
    blue=0,
)
WHITE = ColorRgb(
    red=255,
    green=255,
    blue=255,
)


@dataclass
class LedBehaviour:
    name: str
    class_type: type
    search_str: list[str] | None = None
    color: ColorRgb = WHITE


BEHAVIOUR_REGISTRY = [
    LedBehaviour(
        name="goto",
        class_type=goto.FromBlackboard,
        search_str=None,
        color=YELLOW,
    ),
    LedBehaviour(
        name="cluster",
        class_type=py_trees_ros.action_clients.FromBlackboard,
        search_str=["cluster"],
        color=PURPLE,
    ),
    LedBehaviour(
        name="stabilise",
        class_type=py_trees.timers.Timer,
        search_str=["stabilise"],
        color=ORANGE,
    ),
]

LED_TOPIC = "/auv4/led"


class LedVisitor(VisitorBase):
    def __init__(self, full: bool = False):
        super().__init__(full)

        self.state = {}
        self.previous_state = {}

    def initialise(self) -> None:
        super().initialise()

        self.changed = False
        self.previous_state = self.state

        for led_behaviour in BEHAVIOUR_REGISTRY:
            self.state[led_behaviour.name] = py_trees.common.Status.INVALID

    def run(self, behaviour: py_trees.behaviour.Behaviour) -> None:
        super().run(behaviour)

        for led_behaviour in BEHAVIOUR_REGISTRY:
            if led_behaviour.search_str is None:
                if isinstance(behaviour, led_behaviour.class_type):
                    self.state[led_behaviour.name] = behaviour.status
                    self.changed = self.changed or (
                        self.state[led_behaviour.name]
                        != self.previous_state[led_behaviour.name]
                    )

            else:
                contains_str = False

                cleaned_name = behaviour.name.lower()

                for search_str in led_behaviour.search_str:
                    if search_str in cleaned_name:
                        contains_str = True

                        break

                if contains_str and isinstance(behaviour, led_behaviour.class_type):
                    self.state[led_behaviour.name] = behaviour.status

    def finalise(self) -> None:
        super().finalise()

        for status in self.state.values():
            if status == py_trees.common.Status.SUCCESS:
                self.is_success = True

            if status == py_trees.common.Status.FAILURE:
                self.is_failure = True

                break


def led_handler(
    visitor: LedVisitor,
    led_publisher: Publisher,
    tree: py_trees_ros.trees.BehaviourTree,
) -> None:
    if py_trees.common.Status.FAILURE in visitor.state.values():
        led_publisher.publish(RED)
        return

    if py_trees.common.Status.SUCCESS in visitor.state.values():
        led_publisher.publish(GREEN)
        return

    for led_behaviour in BEHAVIOUR_REGISTRY:
        if led_behaviour.name in visitor.state:
            led_publisher.publish(led_behaviour.color)
            break


def create_led_tree(
    root: py_trees.behaviour.Behaviour,
    unicode_tree_debug=True,
) -> tuple[py_trees_ros.trees.BehaviourTree, Node]:
    node = Node("tree_node")
    led_publisher = node.create_publisher(
        ColorRgb, LED_TOPIC, qos_profile=qos_profile_sensor_data
    )

    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=True)
    tree.snapshot_visitor.display_only_visited_behaviours = True

    led_visitor = LedVisitor()
    tree.add_visitor(led_visitor)

    tree.add_post_tick_handler(
        functools.partial(led_handler, led_visitor, led_publisher)
    )

    return tree, node
