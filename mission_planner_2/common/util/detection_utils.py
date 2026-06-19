from bb_perception_msgs.msg import ClusterPosesRequest
from bb_perception_msgs.srv import (
    ClusterPosesSrv,
    ClusterTfSrv,
    IMPoseEstimatorToggleTemplate,
)
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


def create_cluster_poses_toggle_request(enabled: bool) -> ClusterTfSrv.Request:
    request = ClusterTfSrv.Request()
    request.enabled = enabled
    return request


def create_cluster_poses_srv_request(
    *,
    enabled: bool,
    odom_topic: str = "",
    pose_stamped_topic: str | list[str] = "",
    clustered_child_frame_id: str | list[str] = "",
    sync_queue_size: int = 100,
    sync_tolerance: float = 0.05,
    min_poses: int = 10,
    min_cluster_size: int = 5,
    min_samples: int = 5,
    cluster_selection_epsilon: float = 0.0,
    max_detection_age_s: float = 0.0,
    top_k: int = 1,
    sort_key: int = ClusterPosesRequest.SORT_BY_NUM_CLUSTER_POSES,
    cluster_interval: float = 0.0,
) -> ClusterPosesSrv.Request:
    """Build a ClusterPosesSrv request for the new (spike-less) cluster API.

    On enable, the node bakes in every field from `params`. On disable
    (enabled=False), only `enabled` is read; everything else is ignored.

    When `cluster_interval > 0`, the service re-clusters the poses collected so
    far every `cluster_interval` seconds and publishes a ClusterPoseResultArray
    (the replacement for the removed spike-status feed). Results are ordered by
    `sort_key` and truncated to `top_k`; each is broadcast as
    ``<clustered_child_frame_id>_<i>``.

    Passing a list of `clustered_child_frame_id` (one per pose topic) clusters
    each pose_stamped_topic[i] independently into frame [i]. A single string
    merges every pose topic into one clustering stream.
    """

    def _to_list(v: str | list[str]) -> list[str]:
        return [v] if isinstance(v, str) else list(v)

    request = ClusterPosesSrv.Request()
    request.enabled = enabled
    request.cluster_interval = float(cluster_interval)

    # Drop empty entries so an unset frame falls back to the node default
    # rather than broadcasting under an empty frame id.
    frame_ids = [
        frame_id for frame_id in _to_list(clustered_child_frame_id) if frame_id
    ]

    params = ClusterPosesRequest()
    params.odom_topic = odom_topic
    params.pose_stamped_topics = _to_list(pose_stamped_topic)
    params.clustered_child_frame_ids = frame_ids
    params.sync_queue_size = int(sync_queue_size)
    params.sync_tolerance = float(sync_tolerance)
    params.min_poses = int(min_poses)
    params.min_cluster_size = int(min_cluster_size)
    params.min_samples = int(min_samples)
    params.cluster_selection_epsilon = float(cluster_selection_epsilon)
    params.max_detection_age_s = float(max_detection_age_s)
    params.top_k = int(top_k)
    params.sort_key = int(sort_key)
    request.params = params
    return request
