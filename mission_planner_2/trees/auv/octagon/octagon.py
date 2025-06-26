import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from lifecycle_msgs.srv import ChangeState
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger

from mission_planner_2.commons import cache_tf
from mission_planner_2.commons.blackboard import DynamicSetBlackboard
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
from mission_planner_2.trees.auv.goto import goto

# Generate namespace automatically from file path DONT set manually
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

######################### UPDATE CONSTANTS HERE #########################
VISION_SERVER_TOPIC = "/auv4/trash/manage_nodes"

CAMERA_FRAME = "auv4/front_cam_optical"

LADLE_1_FRAME = "ladle_0"
LADLE_1_FRAME_CLUSTERED = "ladle_0/clustered"
BOTTLE_1_FRAME = "bottle_0"
BOTTLE_1_FRAME_CLUSTERED = "bottle_0/clustered"

LADLE_2_FRAME = "ladle_1"
LADLE_2_FRAME_CLUSTERED = "ladle_1/clustered"
BOTTLE_2_FRAME = "bottle_1"
BOTTLE_2_FRAME_CLUSTERED = "bottle_1/clustered"

LADLE_BASKET_FRAME = "yellow_bucket"
LADLE_BASKET_FRAME_CLUSTERED = "yellow_bucket/clustered"
BOTTLE_BASKET_FRAME = "pink_bucket"
BOTTLE_BASKET_FRAME_CLUSTERED = "pink_bucket/clustered"

FISH_FRAME = "trash/fish"
SHARK_FRAME = "trash/shark"
FISH_FRAME_CLUSTERED = "trash/fish/clustered"
SHARK_FRAME_CLUSTERED = "trash/shark/clustered"
FISH_VIEW_FRAME = "trash/fish/view"
SHARK_VIEW_FRAME = "trash/shark/view"

ACTIVATE_GRABBER = UInt8(data=0)
HALF_CLOSE_GRABBER = UInt8(data=3)

CLUSTER_DURATION = 30
NUM_ROTATIONS = 6
STABILIZE_DURATION = 10
#########################################################################

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_CHOICE_KEY = fk("choice")
_GO_SURFACE_FRAME_KEY = fk("go_surface_frame")
_ACTIVATE_GRABBER_KEY = fk("activate_grabber")
_HALF_CLOSE_GRABBER_KEY = fk("half_close_grabber")
_START_VISION_KEY = fk("bin_start_vision")
_STOP_VISION_KEY = fk("bin_stop_vision")


# TODO might have btr way to do this im just lz to type same thing so many times
def _create_search_tag_goto(num_rotations):
    return goto.FromConstant(
        name="search goto",
        pose=create_stamped_pose(
            frame_id="auv4/base_link_ned",
            yaw=360.0 / num_rotations,
        ),
    )


def create_octagon_root():
    """
    Create the root of the octagon tree.
    """
    root = py_trees.composites.Sequence(
        name="Octagon Root",
        memory=True,
    )

    seq_spoon = py_trees.composites.Sequence(
        name="Spoon Sequence",
        memory=True,
    )

    seq_cup = py_trees.composites.Sequence(
        name="Cup Sequence",
        memory=True,
    )

    srv_start_vision = py_trees_ros.service_clients.FromConstant(
        name="Start vision pipeline",
        service_name=VISION_SERVER_TOPIC,
        service_type=ChangeState,
        service_request=create_start_vision_req(),
        key_response=_START_VISION_KEY,
    )
    check_start_vision_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify start vision pipeline succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=_START_VISION_KEY,
            value=True,
            operator=lambda x, y: x.success == y,
        ),
    )

    srv_get_choice = py_trees_ros.service_clients.FromConstant(
        name="Get choice",
        service_name="/auv4/choice/get_is_fish",
        service_type=Trigger,
        service_request=Trigger.Request(),
        key_response=_CHOICE_KEY,
    )

    set_activate_grabber = py_trees.behaviours.SetBlackboardVariable(
        name="Set claw actuation",
        variable_name=_ACTIVATE_GRABBER_KEY,
        variable_value=ACTIVATE_GRABBER,
        overwrite=True,
    )

    set_half_close_grabber = py_trees.behaviours.SetBlackboardVariable(
        name="Set claw actuation half close",
        variable_name=_HALF_CLOSE_GRABBER_KEY,
        variable_value=HALF_CLOSE_GRABBER,
        overwrite=True,
    )

    pub_activate_grabber = py_trees_ros.publishers.FromBlackboard(
        name="Init grabber",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=_ACTIVATE_GRABBER_KEY,
    )

    par_search_tag = py_trees.composites.Parallel(
        name="Search tag",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )

    seq_rotate_search = py_trees.composites.Sequence(
        name="Search goto seq",
        memory=True,
    )

    seq_rotate_search.add_children(
        [_create_search_tag_goto(NUM_ROTATIONS) for _ in range(NUM_ROTATIONS)]
    )

    cluster_tags = py_trees_ros.actions.ActionClient(
        name="Cluster tags",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=[FISH_FRAME, SHARK_FRAME],
            out_children=[FISH_FRAME_CLUSTERED, SHARK_FRAME_CLUSTERED],
            duration=CLUSTER_DURATION,
            use_cache=False,
            persistent=True,
        ),
    )

    par_search_tag.add_children(
        [
            seq_rotate_search,
            cluster_tags,
        ]
    )

    dynamic_set_surface_pose_frame = DynamicSetBlackboard(
        name="select surface frame",
        key=_CHOICE_KEY,
        update_key=_GO_SURFACE_FRAME_KEY,
        overwrite=True,
        func=lambda choice: (
            create_stamped_pose(FISH_VIEW_FRAME)
            if choice.success
            else create_stamped_pose(SHARK_VIEW_FRAME)
        ),
    )

    # TODO: if the two objects end up to be very similar logic almost the same can put in a sub tree for now this is easier to test each one
    ################## SPOON PART #################
    cluster_spoon = py_trees_ros.actions.ActionClient(
        name="Cluster spoon",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf_multi",
        action_goal=create_clustering_goal(
            in_children=[LADLE_1_FRAME, LADLE_2_FRAME],
            out_children=[LADLE_1_FRAME_CLUSTERED, LADLE_2_FRAME_CLUSTERED],
            duration=CLUSTER_DURATION,
            use_cache=False,
        ),
    )

    # move down to pick up the spoon
    goto_spoon = goto.FromConstant(
        name="Go to spoon",
        pose=create_stamped_pose(
            frame_id=LADLE_1_FRAME_CLUSTERED,
            position_z=-0.1,
        ),
    )

    pub_half_close_grabber = py_trees_ros.publishers.FromBlackboard(
        name="half close spoon",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=_HALF_CLOSE_GRABBER_KEY,
    )

    # now surface with the spoon facing the saved tf
    goto_surface_spoon = goto.FromBlackboard(
        name="Go to surface with spoon",
        pose_key=_GO_SURFACE_FRAME_KEY,
        anchor_frame_name="auv4/base_link_ned",
    )

    cluster_spoon_basket = py_trees_ros.action_clients.FromConstant(
        name="Cluster spoon basket",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf_multi",
        action_goal=create_clustering_goal(
            in_children=LADLE_BASKET_FRAME,
            out_children=LADLE_BASKET_FRAME_CLUSTERED,
            duration=CLUSTER_DURATION,
            use_cache=False,
        ),
    )

    goto_spoon_basket = goto.FromConstant(
        name="Go to spoon basket",
        pose=create_stamped_pose(
            frame_id=LADLE_BASKET_FRAME_CLUSTERED,
        ),
    )

    pub_activate_grabber_spoon = py_trees_ros.publishers.FromBlackboard(
        name="Drop spoon",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=_ACTIVATE_GRABBER_KEY,
    )

    # resurface before start of cup
    # TODO: may need to recluster the surface pose when surface for pts (goto_surface_spoon)
    goto_surface_reset = goto.FromBlackboard(
        name="Go to surface reset",
        pose_key=_GO_SURFACE_FRAME_KEY,
        anchor_frame_name="auv4/base_link_ned",
    )

    cluster_tags_spoon_1 = py_trees_ros.actions.ActionClient(
        name="Cluster tags spoon 1",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=[FISH_FRAME, SHARK_FRAME],
            out_children=[FISH_FRAME_CLUSTERED, SHARK_FRAME_CLUSTERED],
            duration=CLUSTER_DURATION,
            use_cache=False,
            persistent=True,
        ),
    )

    ################## CUP PART #################

    cluster_cup = py_trees_ros.actions.ActionClient(
        name="Cluster cup",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf_multi",
        action_goal=create_clustering_goal(
            in_children=[BOTTLE_1_FRAME, BOTTLE_2_FRAME],
            out_children=[BOTTLE_1_FRAME_CLUSTERED, BOTTLE_2_FRAME_CLUSTERED],
            duration=CLUSTER_DURATION,
            use_cache=False,
        ),
    )

    goto_cup = goto.FromConstant(
        name="Go to cup",
        pose=create_stamped_pose(
            frame_id=BOTTLE_1_FRAME_CLUSTERED,
        ),
    )

    pub_half_close_grabber_2 = py_trees_ros.publishers.FromBlackboard(
        name="half close cup",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=_HALF_CLOSE_GRABBER_KEY,
    )

    # now surface with the cup facing the saved tf
    goto_surface_cup = goto.FromBlackboard(
        name="Go to surface with cup",
        pose_key=_GO_SURFACE_FRAME_KEY,
        anchor_frame_name="auv4/base_link_ned",
    )

    cluster_cup_basket = py_trees_ros.action_clients.FromConstant(
        name="Cluster cup basket",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=BOTTLE_BASKET_FRAME,
            out_children=BOTTLE_BASKET_FRAME_CLUSTERED,
            duration=CLUSTER_DURATION,
            use_cache=False,
        ),
    )

    goto_cup_basket = goto.FromConstant(
        name="Go to cup basket",
        pose=create_stamped_pose(
            frame_id=BOTTLE_BASKET_FRAME_CLUSTERED,
        ),
    )

    # TODO: might not need this since only two objects
    pub_activate_grabber_cup = py_trees_ros.publishers.FromBlackboard(
        name="Drop cup",
        topic_name="/auv4/actuation/input",
        topic_type=UInt8,
        qos_profile=qos_profile_system_default,
        blackboard_variable=_ACTIVATE_GRABBER_KEY,
    )

    # resurface before start of cup
    # TODO: may need to recluster the surface pose when surface for pts (goto_surface_spoon)
    goto_surface_reset_2 = goto.FromBlackboard(
        name="Go to surface reset",
        pose_key=_GO_SURFACE_FRAME_KEY,
        anchor_frame_name="auv4/base_link_ned",
    )

    cluster_tags_cup_1 = py_trees_ros.actions.ActionClient(
        name="Cluster tags cup 1",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=[FISH_FRAME, SHARK_FRAME],
            out_children=[FISH_FRAME_CLUSTERED, SHARK_FRAME_CLUSTERED],
            duration=CLUSTER_DURATION,
            use_cache=False,
            persistent=True,
        ),
    )

    goto_rotation_spoon = goto.FromConstant(
        name="Go to rotation",
        pose=create_stamped_pose(frame_id="auv4/base_link_ned", yaw=180.0),
    )

    goto_rotation_cup = goto.FromConstant(
        name="Go to rotation",
        pose=create_stamped_pose(frame_id="auv4/base_link_ned", yaw=180.0),
    )

    srv_end_vision = py_trees_ros.service_clients.FromConstant(
        name="End vision pipeline",
        service_name=VISION_SERVER_TOPIC,
        service_type=ChangeState,
        service_request=create_end_vision_req(),
        key_response=_STOP_VISION_KEY,
    )
    check_end_vision_succeeded = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Verify end vision pipeline succeeded",
        check=py_trees.common.ComparisonExpression(
            variable=_STOP_VISION_KEY,
            value=True,
            operator=lambda x, y: x.success == y,
        ),
    )

    seq_spoon.add_children(
        children=[
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
            pub_activate_grabber_spoon,
            py_trees.timers.Timer("Wait after drop", duration=STABILIZE_DURATION),
            goto_surface_reset,
            cluster_tags_spoon_1,
        ]
    )

    seq_cup.add_children(
        children=[
            cluster_cup,
            goto_cup,
            py_trees.timers.Timer(
                "Stabilise before pick up", duration=STABILIZE_DURATION
            ),
            pub_half_close_grabber_2,
            py_trees.timers.Timer("Wait after pick up", duration=STABILIZE_DURATION),
            goto_surface_cup,
            cluster_cup_basket,
            goto_cup_basket,
            pub_activate_grabber_cup,
            py_trees.timers.Timer("Wait after drop", duration=STABILIZE_DURATION),
            goto_surface_reset_2,
            cluster_tags_cup_1,
        ]
    )

    root.add_children(
        children=[
            srv_get_choice,
            srv_start_vision,
            check_start_vision_succeeded,
            set_activate_grabber,
            set_half_close_grabber,
            pub_activate_grabber,
            par_search_tag,
            dynamic_set_surface_pose_frame,
            seq_spoon,
            seq_cup,
            goto_rotation_spoon,
            py_trees.timers.Timer(
                "Stabilise after first rotation", duration=STABILIZE_DURATION
            ),
            goto_rotation_cup,
            srv_end_vision,
            check_end_vision_succeeded,
        ]
    )

    return root
