import py_trees
from geometry_msgs.msg import PoseStamped, PoseWithCovariance
from numpy import rad2deg
from transforms3d.euler import quat2euler


# TODO: see if there is a proper way to to do this similar to the template stuff in cpp only supports PoseWithCovariance and PoseStamped
def _extract_pose(pose):
    """
    Extract the position & orientation if avaialable from the pose.
    Pose can be any type of geometry_message
    """
    res = pose
    if isinstance(pose, PoseStamped):
        res = pose.pose
    elif isinstance(pose, PoseWithCovariance):
        res = pose.pose

    return res


def _is_within_threshold(pose, target_pose, threshold):
    """
    Check if the pose is within the threshold of the target pose.
    Pose and target pose must be in NED frame as well world_ned.
    Threshold should be in meters in NED frame [x, y, z, r, p, y]
    """
    # Check if the pose is within the threshold of the target pose
    pose = _extract_pose(pose)
    target_pose = _extract_pose(target_pose)
    p = pose.position
    target_p = target_pose.position

    # Check if the pose is within the threshold of the target pose
    if abs(p.x - target_p.x) > threshold[0]:
        return False
    if abs(p.y - target_p.y) > threshold[1]:
        return False
    if abs(p.z - target_p.z) > threshold[2]:
        return False

    # check orientation in quaternion, input threshold is rpy
    # convert to quaternion
    if pose.orientation:
        quat = (
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        )
        quat_target = (
            target_pose.orientation.w,
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
        )
        q_rpy = rad2deg(quat2euler(quat))
        q_target = rad2deg(quat2euler(quat_target))

        q_rpy = [x % 360 for x in q_rpy]
        q_target = [x % 360 for x in q_target]

        acute_diff = list()

        for curr, target in zip(q_rpy, q_target):
            max_ang = max(curr, target)
            min_ang = min(curr, target)
            acute_diff.append(min(max_ang - min_ang, 360 - (max_ang - min_ang)))

        # TODO: the check here is done in terms of RPY not sure if got diff when using quaternions instead like the old mission planner
        if acute_diff[0] > threshold[3]:
            return False
        if acute_diff[1] > threshold[4]:
            return False
        if acute_diff[2] > threshold[5]:
            return False

    return True


class FromBlackboard(py_trees.behaviour.Behaviour):
    """
    A non-blocking behaviour that checks if the pose is within the threshold of the target pose.

    Args:
        name (str): The name of the behaviour.
        key_pose (str): The key of the blackboard variable to read the pose from.
        key_target_pose (str): The key of the blackboard variable to read the target pose from.
        key_threshold (str): The key of the blackboard variable to read the threshold from. This is a list of 6 values [x, y, z, r, p, y] in meters and degrees.
    """

    def __init__(self, name, key_pose, key_target_pose, key_threshold):
        super().__init__(name)
        self.key_pose = key_pose
        self.key_target_pose = key_target_pose
        self.key_threshold = key_threshold
        self.blackboard = self.attach_blackboard_client(name="check_pose_threshold")
        self.blackboard.register_key(
            key=self.key_pose,
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key=self.key_target_pose,
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key=self.key_threshold,
            access=py_trees.common.Access.READ,
        )

    def update(self) -> py_trees.common.Status:
        pose = self.blackboard.get(self.key_pose)
        target_pose = self.blackboard.get(self.key_target_pose)
        threshold = self.blackboard.get(self.key_threshold)

        if _is_within_threshold(pose, target_pose, threshold):
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE


class FromConstant(py_trees.behaviour.Behaviour):
    """
    A non-blocking behaviour that checks if the pose is within the threshold of the target pose.

    Args:
        name (str): The name of the behaviour.
        key_pose (str): The key of the blackboard variable to read the pose from.
        key_target_pose (str): The key of the blackboard variable to read the target pose from.
        threshold (list): A list of 6 values [x, y, z, r, p, y] in meters and degrees.
    """

    def __init__(self, name, key_pose, key_target_pose, threshold):
        super().__init__(name)
        self.key_pose = key_pose
        self.key_target_pose = key_target_pose
        self.threshold = threshold

        self.blackboard = self.attach_blackboard_client("check within threshold const")

        self.blackboard.register_key(
            self.key_pose,
            access=py_trees.common.Access.READ,
        )

        self.blackboard.register_key(
            self.key_target_pose,
            access=py_trees.common.Access.READ,
        )

    def update(self) -> py_trees.common.Status:
        pose = self.blackboard.get(self.key_pose)
        target_pose = self.blackboard.get(self.key_target_pose)

        if _is_within_threshold(pose, target_pose, self.threshold):
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE
