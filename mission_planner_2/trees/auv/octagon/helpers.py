import numpy as np
from bb_behavior_msgs.action import ControlledSpin
from bb_perception_msgs.srv import GetObjectCount
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3
from std_srvs.srv import Trigger

from mission_planner_2.commons.pose_utils import create_stamped_pose


def get_table_to_symbol_pose(
    choice: Trigger.Response,
    fish_tf: TransformStamped | None,
    shark_tf: TransformStamped | None,
) -> PoseStamped:
    if fish_tf is None or shark_tf is None:
        return create_stamped_pose(frame_id="auv4/base_link_ned")

    target_tf = fish_tf if choice.success else shark_tf

    yaw = np.arctan2(
        target_tf.transform.translation.y, target_tf.transform.translation.x
    )

    target_pose = create_stamped_pose(
        frame_id=target_tf.header.frame_id,
        yaw=yaw,
        use_radians=True,
    )

    return target_pose


def _euclidean_dist(x1, y1, z1, x2, y2, z2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2) ** 0.5


def _is_obj_in_basket(
    obj_pos: Vector3,
    basket_pos: Vector3,
    overlap_threshold: float,
) -> bool:
    if obj_pos is None or basket_pos is None:
        raise ValueError("Object position or basket position is None")

    # FIXME: do x y check separately
    dist = _euclidean_dist(
        obj_pos.x, obj_pos.y, obj_pos.z, basket_pos.x, basket_pos.y, basket_pos.z
    )

    return dist < overlap_threshold


def trash_view_frame_func(
    tf_0: TransformStamped | None,
    tf_1: TransformStamped | None,
    view_frame_0: str,
    view_frame_1: str,
    basket_tf: TransformStamped | None,
    overlap_threshold: float = 0.5,
):
    if tf_0 is None:
        raise ValueError("no ladle clusters found")  # should not reach here
    if basket_tf is None:
        raise ValueError("no basket cluster found")  # should not reach here too

    is_in_basket_0 = _is_obj_in_basket(tf_0, basket_tf, overlap_threshold)

    if not is_in_basket_0:
        return create_stamped_pose(view_frame_0)

    # TODO: if want can change to assertions for these assumptions
    if tf_1 is None:
        raise ValueError(
            "one is in basket one is not found glhf"
        )  # should not reach here

    is_in_basket_1 = _is_obj_in_basket(tf_1, basket_tf, overlap_threshold)

    if not is_in_basket_1:
        return create_stamped_pose(view_frame_1)

    raise ValueError("both are in basket how can it be MaGic")  # should not reach here


def create_spin_goal(response: GetObjectCount.Response):
    return ControlledSpin.Goal(
        yaw_amount=(
            360.0 * (4 - response.num_bottles_on_table - response.num_ladles_on_table)
        ),
        yaw_tolerance=3.0,
        yaw_rate=20.0,
        timeout_seconds=30.0,
    )
