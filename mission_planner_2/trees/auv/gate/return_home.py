import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf

from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
)
from mission_planner_2.trees.auv.goto import goto


NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
CLUSTERING_DURATION = 15
STABILIZE_DURATION = 5.0

GATE_APPROACH_HEIGHT = 0.40

CAMERA_FRAME = "auv4/front_cam_optical"
TEMPLATE_FRAME_YOLO = "gate"
TEMPLATE_FRAME_YOLO_CLUSTERED = "gate/clustered"
GATE_CENTRE_FRAME = "gate/centre"
GATE_AFTER_CENTRE_FRAME = "gate/after_centre"
#########################################################################


def create_return_root():

    # Root sequence
    seq_return_root = py_trees.composites.Sequence(
        name="Return root",
        memory=True,
    )

    # Step 1: Move towards gate
    # TODO: Eventually, we should cache some position after passing through the gate
    #       and return to this position after doing all the tasks
    gate_init_pose = create_stamped_pose("world_ned", position_z=GATE_APPROACH_HEIGHT)
    goto_after_gate = goto.FromConstant(
        "Goto after gate", NAMESPACE, gate_init_pose
    )

    # Step 2: Cluster gate transforms
    action_cluster_gate = py_trees_ros.action_clients.FromConstant(
        name="Cluster gate transforms",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_parent=CAMERA_FRAME,
            in_child=TEMPLATE_FRAME_YOLO,
            out_child=TEMPLATE_FRAME_YOLO_CLUSTERED,
            duration=CLUSTERING_DURATION,
            use_cache=False,
        ),
    )

    # Step 3: Move to after center position to align
    goto_after_gate_center = goto.FromConstant(
        "Goto after gate centre", NAMESPACE, create_stamped_pose(GATE_AFTER_CENTRE_FRAME)
    )

    # Step 4: Wait to stabilize
    timer_stabilize = py_trees.timers.Timer(
        "Stabilize before pass through", STABILIZE_DURATION
    )

    # Step 5: Move to center position, and we are done
    # NOTE: This actually passes through the gate since gate center is
    #       in front of the gate while the AUV will be behind at this point
    goto_gate_centre = goto.FromConstant(
        "Goto gate centre", NAMESPACE, create_stamped_pose(GATE_CENTRE_FRAME)
    )

    # Assemble tree in execution order
    seq_return_root.add_children(
        children=[
            goto_after_gate,
            action_cluster_gate,
            goto_after_gate_center,
            timer_stabilize,
            goto_gate_centre
        ]
    )

    return seq_return_root
