import py_trees
import py_trees_ros
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import qos_profile_system_default
from bb_controls_msgs.srv import Controller
from std_srvs.srv import Trigger

ARM_MAVROS_TOPIC = "/mavros/cmd/arming"
SET_MODE_MAVROS_TOPIC = "/mavros/set_mode"
SET_ALTITUDE_MAVROS_TOPIC = "/mavros/setpoint_position/global"
ENABLE_MINIAUV_CONTROLS = "/mini/controls/controller"
ALTITUDE_KEY = "/miniauv/altitude"
TRIGGER_MINICONTROLLER = "/mini/controls/trigger"
STABILIZE_DURATION = 10.0


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
        service_request=CommandBool.Request(value = True),
    )

    srv_set_mode_mavros = py_trees_ros.service_clients.FromConstant(
        name="Set Mode Mavros",
        service_type=SetMode,
        service_name=SET_MODE_MAVROS_TOPIC,
        service_request=SetMode.Request(base_mode = 0, custom_mode = "ALT_HOLD"),
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
        qos_profile=qos_profile_system_default,
        blackboard_variable=ALTITUDE_KEY,
    )

    enable_controls = py_trees_ros.service_clients.FromConstant(
        name="Enable controls for miniauv",
        service_type=Controller,
        service_name=ENABLE_MINIAUV_CONTROLS,
        service_request=Controller.Request(enable=True, pause=False, disable_altitude=True),
    )

    timer_stabilize = py_trees.timers.Timer(
        "Stabilize before enabling thrust allocator", STABILIZE_DURATION
    )
    enable_minicontroller = py_trees_ros.service_clients.FromConstant(
        name="Enable thrust allocator for miniauv",
        service_type=Trigger,
        service_name=TRIGGER_MINICONTROLLER,
        service_request=Trigger.Request(),
    )

    root.add_children(
        [
            srv_arm_mavros,
            srv_set_mode_mavros,
            set_altitude_mavros,
            pub_altitude_mavros,
            enable_controls,
            timer_stabilize,
            enable_minicontroller
        ]
    )

    return root
