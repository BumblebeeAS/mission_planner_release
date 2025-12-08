#!/usr/bin/env python3
import py_trees

from mission_planner_2.common.util.namespace_utils import generate_namespace
from mission_planner_2.common.util.pose_utils import create_stamped_pose
from mission_planner_2.vehicles.auv.trees.robosub24.goto import goto

# Generate namespace automatically from file path
NAMESPACE = generate_namespace()


def create_test_convert_service_root():
    """
    Creates a simple test tree to test the pose conversion service.

    Sequence:
    1. Go to (0, 0, 1) in world_ned
    2. Go to (0, 0, 0) in test_frame
    3. Return to (0, 0, 1) in world_ned
    4. Go to (0, 0, 0) in test_frame with anchor frame "auv4/front_cam_optical"
    """
    root = py_trees.composites.Sequence(
        name="Test Conversion Service",
        memory=True,
    )

    # Create poses
    start_pose = create_stamped_pose(
        "world_ned",  # Frame ID
        position_x=0.0,
        position_y=0.0,
        position_z=1.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
    )

    test_frame_pose = create_stamped_pose(
        "test_frame",  # Frame ID
        position_x=0.0,
        position_y=0.0,
        position_z=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
    )

    # Create goto behaviors
    goto_start = goto.FromConstant(
        name="Goto Start Position (0, 0, 1) in world_ned",
        parent_namespace=NAMESPACE,
        pose=start_pose,
    )

    goto_test_frame = goto.FromConstant(
        name="Goto (0, 0, 0) in test_frame",
        parent_namespace=NAMESPACE,
        pose=test_frame_pose,
    )

    goto_start_again = goto.FromConstant(
        name="Return to Start Position (0, 0, 1) in world_ned",
        parent_namespace=NAMESPACE,
        pose=start_pose,
    )

    goto_test_frame_with_anchor = goto.FromConstant(
        name="Goto (0, 0, 0) in test_frame with anchor frame",
        parent_namespace=NAMESPACE,
        pose=test_frame_pose,
        anchor_frame_name="auv4/front_cam_optical",  # Specify anchor frame
    )

    # Add behaviors to the root sequence
    root.add_children(
        [
            goto_start,
            py_trees.timers.Timer(name="Stabilize (30 seconds)", duration=30.0),
            goto_test_frame,
            py_trees.timers.Timer(name="Stabilize (30 seconds)", duration=30.0),
            goto_start_again,
            py_trees.timers.Timer(name="Stabilize (30 seconds)", duration=30.0),
            goto_test_frame_with_anchor,
        ]
    )

    return root
