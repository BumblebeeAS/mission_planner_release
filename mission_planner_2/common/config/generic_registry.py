import typing
from dataclasses import dataclass
from enum import Enum


@dataclass
class ActionRegistry:
    topic: str
    type: typing.Any


@dataclass
class ServiceRegistry:
    topic: str
    type: typing.Any


# For typing
class SharedAction(Enum):
    pass


class SharedService(Enum):
    pass
