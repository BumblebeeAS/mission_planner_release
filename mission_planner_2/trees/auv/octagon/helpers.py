import numpy as np
from geometry_msgs.msg import TransformStamped, Vector3
from mission_planner_2.commons.pose_utils import create_stamped_pose
from std_srvs.srv import SetBool


def get_table_to_surface_target_yaw(
    choice: SetBool.Response,
    fish_tf: TransformStamped,
    shark_tf: TransformStamped,
    table_tf: TransformStamped,
) -> float:
    target_tf = fish_tf if choice.success else shark_tf
    x_offset = target_tf.transform.translation.x - table_tf.transform.translation.x
    y_offset = target_tf.transform.translation.y - table_tf.transform.translation.y
    return np.arctan2(y_offset, x_offset)


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
