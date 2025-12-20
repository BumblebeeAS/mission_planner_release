#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Callable

import py_trees
import py_trees_ros
from bb_auv_msgs.msg import ColorRgb
from py_trees.visitors import VisitorBase
from rclpy.publisher import Publisher

from mission_planner_2.vehicles.auv.trees.robosub24.goto import goto

RED = ColorRgb(red=255, green=0, blue=0)
GREEN = ColorRgb(red=0, green=255, blue=0)
BLUE = ColorRgb(red=0, green=0, blue=255)
YELLOW = ColorRgb(red=255, green=255, blue=0)
PURPLE = ColorRgb(red=255, green=0, blue=255)
CYAN = ColorRgb(red=0, green=255, blue=255)
ORANGE = ColorRgb(red=255, green=128, blue=0)
WHITE = ColorRgb(red=255, green=255, blue=255)


@dataclass
class LedBehaviour:
    """A data class to hold LED behavior configuration."""

    name: str
    color: ColorRgb
    match_func: Callable[[py_trees.behaviour.Behaviour], bool]


BEHAVIOUR_REGISTRY = {
    "goto": LedBehaviour(
        name="goto",
        color=YELLOW,
        match_func=lambda x: isinstance(x, goto.FromBlackboard),
    ),
    "cluster": LedBehaviour(
        name="cluster",
        color=PURPLE,
        match_func=lambda x: (
            isinstance(x, py_trees_ros.action_clients.FromBlackboard)
            and "cluster" in x.name.lower()
        ),
    ),
}


class LedVisitor(VisitorBase):

    def __init__(self, full: bool = False):
        super().__init__(full)

        self.state = {
            name: py_trees.common.Status.INVALID for name in BEHAVIOUR_REGISTRY
        }
        self.previous_state = self.state.copy()
        self.running_behavior_color = None

    def initialise(self) -> None:
        super().initialise()

        self.changed = False
        self.previous_state = self.state.copy()
        self.state = {
            name: py_trees.common.Status.INVALID for name in BEHAVIOUR_REGISTRY
        }
        self.running_behavior_color = None

    def run(self, behaviour: py_trees.behaviour.Behaviour) -> None:
        super().run(behaviour)

        for name, led_behaviour in BEHAVIOUR_REGISTRY.items():
            if led_behaviour.match_func(behaviour):
                self.state[name] = behaviour.status
                return

    def finalise(self) -> None:
        super().finalise()

        for name, current_status in self.state.items():
            previous_status = self.previous_state[name]

            if (
                current_status == py_trees.common.Status.RUNNING
                and previous_status != py_trees.common.Status.RUNNING
            ):
                self.running_behavior_color = BEHAVIOUR_REGISTRY[name].color
                break


def led_handler(
    visitor: LedVisitor,
    led_publisher: Publisher,
    tree: py_trees_ros.trees.BehaviourTree,
) -> None:
    if visitor.running_behavior_color:
        led_publisher.publish(visitor.running_behavior_color)
