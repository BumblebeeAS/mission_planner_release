import py_trees
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool

from mission_planner_2.commons import service_clients
from mission_planner_2.commons.blackboard import (
    DynamicSetBlackboard,
    full_key_generator,
)
from mission_planner_2.commons.namespace_utils import generate_namespace
from mission_planner_2.commons.pose_utils import create_stamped_pose
from mission_planner_2.trees.auv.goto import goto_node

# Define a main namespace for the test
NAMESPACE = "/auv4/test_namespacing"
NAMESPACE = generate_namespace()
fk = full_key_generator(NAMESPACE)


def _create_test_pose(x, y, z, yaw=0.0):
    """Create a test pose for goto operations."""
    return create_stamped_pose(
        frame_id="world_ned",
        position_x=x,
        position_y=y,
        position_z=z,
        roll=0.0,
        pitch=0.0,
        yaw=yaw,
    )


def _transform_pose(pose: PoseStamped):
    """Simple function to transform a pose (adds 1.0 to x position for testing)."""
    new_pose = PoseStamped()
    new_pose.header = pose.header
    new_pose.pose = pose.pose
    new_pose.pose.position.x += 1.0
    return new_pose


def create_namespacing_test_root():
    """
    Creates a test tree to verify namespacing capabilities in the mission planner.

    The tree tests:
    1. Setting blackboard variables with proper namespacing
    2. Using goto_node with namespacing
    3. Using service clients with namespacing
    4. Using DynamicSetBlackboard to update values based on input
    5. Testing service clients from blackboard

    Uses only the root namespace defined at the top of the file.
    """
    print(f"Test Namespacing: {NAMESPACE}")
    root = py_trees.composites.Sequence(
        name="Namespacing Test Root",
        memory=True,
    )

    # Test 1: Setting and using blackboard variables with namespacing
    test_pose = _create_test_pose(1.0, 2.0, 1.5, -90.0)

    # Set poses in namespace
    set_pose = py_trees.behaviours.SetBlackboardVariable(
        name="Set Pose",
        variable_name=fk("test_pose"),
        variable_value=test_pose,
        overwrite=True,
    )

    # Test 2: Using goto_node with namespace
    goto = goto_node.FromBlackboard(
        name="Goto",
        parent_namespace=NAMESPACE,
        pose_key="test_pose",
    )

    # Test 3: Using FromConstant service client with namespacing
    # Create a dummy service request for testing
    set_bool_request = SetBool.Request()
    set_bool_request.data = True

    service = service_clients.FromConstant(
        name="Service",
        namespace=NAMESPACE,
        service_type=SetBool,
        service_name="/auv4/test_service",
        service_request=set_bool_request,
        key_response="service_response",
    )

    # Test 4: Service clients from blackboard (testing request on blackboard)

    # Define the key for service request
    req_key = "set_bool_request"

    # Set the request on the blackboard with proper namespacing
    set_req = py_trees.behaviours.SetBlackboardVariable(
        name="Set Request",
        variable_name=fk(req_key),
        variable_value=set_bool_request,
        overwrite=True,
    )

    # Create service client that reads the request from the blackboard
    service_from_bb = service_clients.FromBlackboard(
        name="Service From BB",
        namespace=NAMESPACE,
        service_type=SetBool,
        service_name="/auv4/test_service",
        key_request=req_key,
        key_response="set_bool_response",
    )

    # Check if the response exists in the proper namespace
    check_resp = py_trees.behaviours.CheckBlackboardVariableExists(
        name="Check Response",
        variable_name=fk("set_bool_response"),
    )

    # Test 5: Check if FromConstant goto works with namespacing
    goto_const = goto_node.FromConstant(
        name="Goto Constant",
        parent_namespace=NAMESPACE,
        pose=_create_test_pose(5.0, 5.0, 2.5, 0.0),
    )

    # Test 6: Using DynamicSetBlackboard to update values with proper namespacing
    dynamic_update = DynamicSetBlackboard(
        name="Dynamic Update",
        namespace=NAMESPACE,
        key="test_pose",
        update_key="transformed_pose",
        overwrite=True,
        func=_transform_pose,
    )

    # Check if a blackboard variable exists in the correct namespace
    check_var = py_trees.behaviours.CheckBlackboardVariableExists(
        name="Check Var",
        variable_name=fk("transformed_pose"),
    )

    # Final test: Use the transformed pose for goto operation
    goto_transformed = goto_node.FromBlackboard(
        name="Goto Transformed",
        parent_namespace=NAMESPACE,
        pose_key="transformed_pose",
    )

    # Add all tests to the root
    root.add_children(
        [
            set_pose,
            goto,
            service,
            set_req,
            service_from_bb,
            check_resp,
            goto_const,
            dynamic_update,
            check_var,
            goto_transformed,
        ]
    )

    return root
