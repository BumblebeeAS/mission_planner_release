import py_trees
import py_trees_ros

from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.miniauv.goto import goto
from mission_planner_2.trees.miniauv.utils.controls_init import (
    create_init_controls_root,
)
from bb_perception_msgs.action import ClusterTf
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mission_planner_2.commons.detection_utils import (
    create_end_vision_req,
    create_start_vision_req,
)
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import (
    create_clustering_goal,
    create_stamped_pose,
)

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)
######################### UPDATE CONSTANTS HERE #########################
VISION_SERVER_TOPIC = "/auv4/gate_front/manage_nodes"
CLUSTERING_DURATION = 20
STABILIZE_DURATION = 30.0

GATE_APPROACH_HEIGHT = 0.40
FORWARD_DISTANCE = 3.0
BASE_LINK_FRAME = "orca4_ned"
FIXED_DEPTH = -2.0
CAMERA_FRAME = "front_camera_frame"
TEMPLATE_FRAME_YOLO = "gate"
TEMPLATE_FRAME_YOLO_CLUSTERED = "gate/clustered"
CLUSTERED_OUT_PARENT = "map"
GATE_CENTRE_FRAME = "gate/centre/view"
TRIGGER_MINICONTROLLER = "/mini/controls/trigger"

#########################################################################

def create_goto_test():
    root = py_trees.composites.Sequence(name="Goto test", memory=True)

    seq_controls_init = create_init_controls_root(FIXED_DEPTH)

    timer_stabilize_stationkeep = py_trees.timers.Timer(
        "Stabilize before task", STABILIZE_DURATION
    )

    """ 
    Assumptions:
    1. Gate is right infront of me
    2. Vision node is activated by launch file (Configured to active, no need to toggle the service)
    """

    action_cluster_gate = py_trees_ros.action_clients.FromConstant(
        name="Cluster gate transforms",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=TEMPLATE_FRAME_YOLO,
            out_children=TEMPLATE_FRAME_YOLO_CLUSTERED,
            out_parents=CLUSTERED_OUT_PARENT,
            duration=CLUSTERING_DURATION,
            use_cache=False,
        ),
    )

    goto_see_pictures = goto.FromConstant(
        "Goto picture position", create_stamped_pose(GATE_CENTRE_FRAME)
    )

    timer_stabilize_main = py_trees.timers.Timer(
        "Stabilize before task", STABILIZE_DURATION
    )

    test_pose = create_stamped_pose(frame_id=BASE_LINK_FRAME, position_x=5.0)

    goto_action = goto.FromConstant(name="Goto one pose", pose=test_pose)

    root.add_children(
        [
            seq_controls_init,
            action_cluster_gate,
            timer_stabilize_stationkeep,
            goto_see_pictures,
            timer_stabilize_main,
            goto_action
        ]
    )

    return root
