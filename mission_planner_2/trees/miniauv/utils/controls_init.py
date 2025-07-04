import py_trees
import py_trees_ros
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.srv import CommandBool, SetMode

ARM_MAVROS_TOPIC = "/mavros/cmd/arming"
SET_MODE_MAVROS_TOPIC = "/mavros/set_mode"
SET_ALTITUDE_MAVROS_TOPIC = "/mavros/setpoint_position/global"
ALTITUDE_KEY = "/miniauv/altitude"


def create_altitude_to_pub(depth):
    out = GeoPoseStamped()
    out.pose.position.altitude = depth
    return out


def create_init_controls_root(depth):
    root = py_trees.composites.Sequence(
        name="Initialize Controls",
        memory=True,
    )

    srv_arm_mavros = py_trees_ros.service_clients.FromConstant(
        name="Arm Miniauv Mavros",
        service_type=CommandBool,
        service_name=ARM_MAVROS_TOPIC,
        service_request=CommandBool.Request(),
    )

    srv_set_mode_mavros = py_trees_ros.service_clients.FromConstant(
        name="Set Mode Mavros",
        service_type=SetMode,
        service_name=SET_MODE_MAVROS_TOPIC,
        service_request=SetMode.Request(),
    )

    set_altitude_mavros = py_trees.behaviours.SetBlackboardVariable(
        name="Set altitude variable",
        variable_name=ALTITUDE_KEY,
        variable_value=create_altitude_to_pub(depth),
        overwrite=True,
    )

    pub_altitude_mavros = py_trees_ros.publishers.FromBlackboard(
        name="Publish altitude setpoint",
        topic_name=SET_ALTITUDE_MAVROS_TOPIC,
        topic_type=GeoPoseStamped,
        blackboard_variable=ALTITUDE_KEY,
    )

    root.add_children(
        [srv_arm_mavros, srv_set_mode_mavros, set_altitude_mavros, pub_altitude_mavros]
    )

    return root
