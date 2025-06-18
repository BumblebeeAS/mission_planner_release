from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState


def create_start_vision_req() -> ChangeState.Request:
    request = ChangeState.Request()
    request.transition = Transition()
    request.transition.id = Transition.TRANSITION_ACTIVATE
    request.transition.label = "TRANSITION_ACTIVATE"
    return request


def create_end_vision_req() -> ChangeState.Request:
    request = ChangeState.Request()
    request.transition = Transition()
    request.transition.id = Transition.TRANSITION_DEACTIVATE
    request.transition.label = "TRANSITION_DEACTIVATE"
    return request
