import py_trees
import py_trees_ros
from bb_perception_msgs.action import ClusterTf
from mission_planner_2.commons.blackboard import DynamicSetBlackboard
from mission_planner_2.commons.namespace_utils import (
    full_key_generator,
    generate_namespace,
)
from mission_planner_2.commons.pose_utils import create_clustering_goal
from mission_planner_2.commons.tf_checker import create_tf_checker_from_constant_root
from mission_planner_2.trees.auv.goto import goto
from mission_planner_2.trees.auv.octagon.helpers import view_frame_func

NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)

# THESE KEYS ARE USED INTERNALLY FOR THIS TASK AND SHOULD NOT NEED TO BE CHANGED UNLESS THEY CLASH
# DONT go move it in the section to be updated
_FISH_TF_KEY = fk("fish_tf")
_SHARK_TF_KEY = fk("shark_tf")
_GO_SURFACE_FRAME_KEY = fk("surface_frame")


def create_reset_after_rubbish_root(
    fish_frame: str = "fish",
    fish_frame_clustered: str = "fish/clustered",
    shark_frame: str = "shark",
    shark_frame_clustered: str = "shark/clustered",
    fish_view_frame: str = "fish_view",
    fish_view_frame_hardcoded: str = "fish_view_hardcoded",
    shark_view_frame: str = "shark_view",
    shark_view_frame_hardcoded: str = "shark_view_hardcoded",
    cluster_duration: int = 10,
    choice_key: str = "choice",
    rubbish_name: str = "spoon 1",
):
    root = py_trees.composites.Sequence(
        name=f"Reset after rubbish ({rubbish_name})",
        memory=True,
    )

    # TODO: add the search seq here a bit laze now can be done later

    cluster_reset_symbols_1 = py_trees_ros.actions.ActionClient(
        name=f"Cluster symbols ({rubbish_name})",
        action_type=ClusterTf,
        action_name="/auv4/cluster_tf",
        action_goal=create_clustering_goal(
            in_children=[fish_frame, shark_frame],
            out_children=[fish_frame_clustered, shark_frame_clustered],
            duration=cluster_duration,
            use_cache=False,
            persistent=True,
        ),
    )

    symbol_tf_checker = create_tf_checker_from_constant_root(
        start_frames=[fish_frame_clustered, shark_frame_clustered],
        update_keys=[_FISH_TF_KEY, _SHARK_TF_KEY],
        end_frames=["world_ned", "world_ned"],
        fallback_val=[fish_view_frame_hardcoded, shark_view_frame_hardcoded],
    )

    dynamic_set_surface_pose_frame = DynamicSetBlackboard(
        name=f"Select surface frame ({rubbish_name})",
        key=[choice_key, _FISH_TF_KEY, _SHARK_TF_KEY],
        update_key=_GO_SURFACE_FRAME_KEY,
        overwrite=True,
        func=lambda choice, fish_tf, shark_tf: view_frame_func(
            choice, fish_tf, shark_tf, fish_view_frame, shark_view_frame
        ),
    )

    goto_view_frame = goto.FromBlackboard(
        name=f"Go to view frame ({rubbish_name})",
        pose_key=_GO_SURFACE_FRAME_KEY,
    )

    root.add_children(
        children=[
            cluster_reset_symbols_1,
            symbol_tf_checker,
            dynamic_set_surface_pose_frame,
            goto_view_frame,
        ]
    )

    return root
