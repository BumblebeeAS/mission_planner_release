#!/usr/bin/env python3

import py_trees
from py_trees.behaviour import Behaviour
from py_trees_ros import conversions, utilities
from py_trees_ros_interfaces.msg import BehaviourTree
from py_trees_ros_interfaces.srv import OpenSnapshotStream, CloseSnapshotStream
from unique_identifier_msgs.msg import UUID

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TreeMonitor(Node):
    """
    Prototype node for flashing a different LED colour based on currently executing behavior.

    Run this node **strictly after** running the behavior tree.
    """
    def __init__(self):
        super().__init__("tree_monitor")

        self.led_pub = self.create_publisher(String, "led/input", 10)

        self.open_client = self.create_client(
            OpenSnapshotStream, 
            "/tree/snapshot_streams/open"
        )

        future = self.open_client.call_async(OpenSnapshotStream.Request())
        rclpy.spin_until_future_complete(self, future)
        response: OpenSnapshotStream.Response = future.result()
        self.topic_name = response.topic_name

        self.snapshot_sub = self.create_subscription(
            BehaviourTree,
            self.topic_name,
            self.snapshot_callback,
            utilities.qos_profile_latched()
        )

        self.close_client = self.create_client(
            CloseSnapshotStream,
            "/tree/snapshot_streams/close"
        )

    def shutdown(self):
        if self.close_client is None:
            return
        req = CloseSnapshotStream.Request(topic_name=self.topic_name)
        future = self.close_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        _ = future.result()
        self.get_logger().info("snapshot_closed")

    def snapshot_callback(self, msg: BehaviourTree) -> None:
        serialised_behaviours = {}
        root_id = None
        for serialised_behaviour in msg.behaviours:
            if serialised_behaviour.parent_id == UUID():
                root_id = conversions.msg_to_uuid4(serialised_behaviour.own_id)
            serialised_behaviours[
                conversions.msg_to_uuid4(serialised_behaviour.own_id)
            ] = serialised_behaviour
    
        def deserialise_tree_recursively(msg):
            behaviour = conversions.msg_to_behaviour(msg)
            for serialised_child_id in msg.child_ids:
                child_id = conversions.msg_to_uuid4(serialised_child_id)
                child = deserialise_tree_recursively(
                    serialised_behaviours[child_id]
                )
                # invasive hack to revert the dummy child we added in msg_to_behaviour
                if isinstance(behaviour, py_trees.decorators.Decorator):
                    behaviour.children = [child]
                    behaviour.decorated = behaviour.children[0]
                else:
                    behaviour.children.append(child)
                child.parent = behaviour
            if behaviour.children and msg.current_child_id != UUID():
                current_child_id = conversions.msg_to_uuid4(msg.current_child_id)
                for index, child in enumerate(behaviour.children):
                    if child.id == current_child_id:
                        if isinstance(behaviour, py_trees.composites.Sequence):
                            behaviour.current_index = index
                        behaviour.current_child = child
                        break
            return behaviour

        root = deserialise_tree_recursively(serialised_behaviours[root_id])

        tip_name = root.tip().name.lower()
        self.get_logger().info("the tip is currently: " + tip_name)
        if "goto" in tip_name:
            self.led_pub.publish(String(data="ff8243"))
        elif "wait" in tip_name:
            self.led_pub.publish(String(data="fce205"))


def main(args=None):
    rclpy.init(args=args)
    node = TreeMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("stopped")
    except Exception as e:
        node.get_logger().fatal(f"stopped with exception: {e}")
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
