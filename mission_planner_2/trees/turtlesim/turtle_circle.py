import py_trees
import py_trees_ros.trees

from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from std_srvs.srv import Empty
from turtlesim.msg import Pose

import mission_planner_2


class TurtleMove(py_trees.behaviour.Behaviour):

    def __init__(self, name, x_vel, z_vel):
        super().__init__(name)
        self.x = x_vel
        self.z = z_vel

    def setup(self, **kwargs):
        self.logger.debug(f"{self.qualified_name}.setup()")
        try:
            self.node: Node = kwargs["node"]
        except:
            raise KeyError(f"{self.qualified_name} did not find 'node' in kwargs")
        self.pub = self.node.create_publisher(
            msg_type=Twist,
            topic="turtle1/cmd_vel",
            qos_profile=py_trees_ros.utilities.qos_profile_latched()
        )

    def update(self) -> py_trees.common.Status:
        self.logger.debug(f"{self.qualified_name}.update()")
        self.pub.publish(Twist(
            linear=Vector3(x=self.x, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=self.z)
        ))
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status):
        self.logger.debug(f"{self.qualified_name}.terminate({self.status}->{new_status})")


def create_turtle_circle_root() -> py_trees.behaviour.Behaviour:
    topics2bb = py_trees.composites.Sequence(
        name="topics2bb",
        memory=True
    )
    pose2bb = py_trees_ros.subscribers.ToBlackboard(
        name="pose2bb",
        topic_name="turtle1/pose",
        topic_type=Pose,
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
        blackboard_variables={"pose": None}
    )
    tasks = py_trees.composites.Sequence(
        name="tasks",
        memory=True)
    wait_for_pose = py_trees.behaviours.WaitForBlackboardVariable(
        name="wait_for_pose",
        variable_name="pose"
    )
    def guard(blackboard: py_trees.blackboard.Blackboard) -> bool:
        return blackboard.pose.x <= 10.0
    test_move = py_trees.decorators.EternalGuard(
        name="check_valid_pose",
        child=TurtleMove(name="move", x_vel=0.05, z_vel=0.05),
        condition=guard,
        blackboard_keys={"pose"}
    )
    move_a_bit = py_trees.decorators.Repeat(
        name="repeat to move a bit",
        child=test_move,
        num_success=1000
    )
    stop = TurtleMove(name="stop", x_vel=0.0, z_vel=0.0)
    reset = mission_planner_2.service_clients.FromConstant(
        name="reset",
        service_type=Empty,
        service_name="/reset",
        service_request=Empty.Request()
    )
    root = py_trees.composites.Parallel(
        name="draw_circle",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            [tasks],
            synchronise=False
        )
    )
    root.add_child(topics2bb)
    topics2bb.add_child(pose2bb)
    root.add_child(tasks)
    tasks.add_children([wait_for_pose, move_a_bit, stop, reset])
    return root
