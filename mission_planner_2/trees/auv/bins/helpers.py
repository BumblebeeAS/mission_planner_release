import numpy as np
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion

from mission_planner_2.commons.pose_utils import create_stamped_pose


def find_acute_angle(pose: PoseStamped) -> PoseStamped:
    r, p, y = euler_from_quaternion(
        [
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]
    )

    y = y - np.pi if y > np.pi / 2 else y

    return create_stamped_pose(
        frame_id=pose.header.frame_id,
        position_x=pose.pose.position.x,
        position_y=pose.pose.position.y,
        position_z=pose.pose.position.z,
        roll=r,
        pitch=p,
        yaw=y,
        use_radians=True,
    )
