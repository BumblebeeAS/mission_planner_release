from mission_planner_2.commons.pose_utils import create_stamped_pose


def view_frame_func(choice, fish_tf, shark_tf, fish_view_frame, shark_view_frame):
    """
    We will take it that we have at least a fish or shark symbol here
    """
    if choice.success:  # is fish
        if isinstance(fish_tf, str):
            return create_stamped_pose(fish_tf)
        return create_stamped_pose(fish_view_frame)

    if isinstance(shark_tf, str):
        return create_stamped_pose(shark_tf)
    return create_stamped_pose(shark_view_frame)
