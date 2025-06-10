import operator

import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from bb_perception_msgs.srv import IMPoseEstimatorToggleTemplate
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger

from mission_planner_2.commons import cache_tf
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import (
    create_clustering_goals,
    create_stamped_pose,
)
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.torpedo.move_to_task import create_move_to_task_root
from mission_planner_2.trees.auv.torpedo.tf_selector import create_tf_selector_root

# Generate namespace automatically from file path DONT set manually
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
TOGGLE_TEMPLATE_TOPIC = "/auv4/front_cam/image_matching/toggle_template"
TEMPLATE_NAME = "Task04_Tagging_02.png"

CAMERA_FRAME = "auv4/front_cam_optical"
TEMPLATE_FRAME_OPTICAL = "Task04_Tagging_02_optical"
TEMPLATE_FRAME_OPTICAL_CLUSTERED = "torpedo_2/clustered"

SPOON_FRAME = "spoon"
SPOON_FRAME_CLUSTERED = "spoon/clustered"
CUP_FRAME = "cup"
CUP_FRAME_CLUSTERED = "cup/clustered"

SPOON_BASKET_FRAME = "spoon_basket"
SPOON_BASKET_FRAME_CLUSTERED = "spoon_basket/clustered"

CUP_BASKET_FRAME = "cup_basket"
CUP_BASKET_FRAME_CLUSTERED = "cup_basket/clustered"

ACTIVATE_GRABBER = UInt8(data=0)
HALF_CLOSE_GRABBER = UInt8(data=3)

CLUSTER_DURATION = 30
STABILIZE_DURATION = 10
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_CHOICE_KEY = fk("choice")
_POSE_KEY = fk("pose")
_GO_SURFACE_POSE_KEY = fk("go_surface_pose")


def create_octagon_root():
    """
    Create the root of the octagon tree.
    """
    root = py_trees.composites.Sequence(
        name="Octagon Root",
        memory=True,
    )

    srv_get_choice = py_trees_ros.service_clients.FromConstant(
        name="Get choice",
        service_name="/auv4/choice/get_is_fish",
        service_type=Trigger,
        service_request=Trigger.Request(),
        key_response=fk(_CHOICE_KEY),
    )

    set_activate_grabber = py_trees.behaviours.SetBlackboardVariable(
        name="Set claw actuation",
        variable_name=fk("grabber_activate"),
        variable_value=ACTIVATE_GRABBER,
        overwrite=True,
    )

    pub_activate_grabber = py_trees_ros.publishers.FromBlackboard(
        name="grab spoon",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=fk("grabber_activate"),
    )

    set_half_close_grabber = py_trees.behaviours.SetBlackboardVariable(
        name="Set claw actuation half close",
        variable_name=fk("grabber_half_close"),
        variable_value=HALF_CLOSE_GRABBER,
        overwrite=True,
    )

    # TODO: search for the tags

    cluster_spoon = py_trees_ros.actions.ActionClient(
        name="Cluster spoon",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goals(
            in_children=SPOON_FRAME,
            out_children=SPOON_FRAME_CLUSTERED,
            duration=CLUSTER_DURATION,
            use_cache=False,
        ),
    )

    # assuming can see the spoon on the table from here if not need to move down first
    cache_tf_surface = cache_tf.ToBlackboard(
        name="Cache tf surface",
        variable_name=fk(_GO_SURFACE_POSE_KEY),
        start=SPOON_FRAME_CLUSTERED,
        end="auv4/base_link_ned",
    )

    seq_spoon = py_trees.composites.Sequence(
        name="Spoon Sequence",
        memory=True,
    )

    seq_cup = py_trees.composites.Sequence(
        name="Cup Sequence",
        memory=True,
    )

    # move down to pick up the spoon
    goto_spoon = goto.FromConstant(
        name="Go to spoon",
        pose=create_stamped_pose(
            frame_id=SPOON_FRAME_CLUSTERED,
        ),
    )

    pub_half_close_grabber = py_trees_ros.publishers.FromBlackboard(
        name="half close spoon",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=fk("grabber_half_close"),
    )

    # now surface with the spoon facing the saved tf
    goto_surface_spoon = goto.FromBlackboard(
        name="Go to surface with spoon",
        pose_key=fk(_GO_SURFACE_POSE_KEY),
        anchor_frame_name="auv4/base_link_ned",
    )

    cluster_spoon_basket = py_trees_ros.action_clients.FromConstant(
        name="Cluster spoon basket",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goals(
            in_children=SPOON_BASKET_FRAME,
            out_children=SPOON_BASKET_FRAME_CLUSTERED,
            duration=CLUSTER_DURATION,
            use_cache=False,
        ),
    )

    goto_spoon_basket = goto.FromConstant(
        name="Go to spoon basket",
        pose=create_stamped_pose(
            frame_id=SPOON_BASKET_FRAME_CLUSTERED,
        ),
    )

    root.add_children(
        children=[
            srv_get_choice,
            set_activate_grabber,
            set_half_close_grabber,
            cache_tf_surface,
            cluster_spoon,
            goto_spoon,
            py_trees.timers.Timer(
                "Stabilise before pick up", duration=STABILIZE_DURATION
            ),
            pub_half_close_grabber,
            py_trees.timers.Timer("Wait after pick up", duration=STABILIZE_DURATION),
            goto_surface_spoon,
            cluster_spoon_basket,
            goto_spoon_basket,
            pub_activate_grabber,
        ]
    )

    return root
