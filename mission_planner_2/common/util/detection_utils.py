from bb_perception_msgs.srv import IMPoseEstimatorToggleTemplate
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


def create_img_matching_request(
    enable: bool,
    camera_frame_id: str,
    template_name: str,
) -> IMPoseEstimatorToggleTemplate.Request:
    request = IMPoseEstimatorToggleTemplate.Request()
    request.enable = enable
    request.camera_frame_id = camera_frame_id
    request.template_name = template_name

    return request
