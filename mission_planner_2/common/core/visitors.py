import collections
import re
import typing

import py_trees
from py_trees import blackboard, behaviour, display
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String

class LoggingSnapshotVisitor(py_trees.visitors.SnapshotVisitor):
    """
    Visit the tree, capturing the visited path, it's changes since the last tick.

    Additionally *logs* the snapshot to console.

    Args:
        node: The ROS2 node to obtain a logger from
        display_only_visited_behaviours: useful for cropping the unvisited part of a large tree
        display_blackboard: print to the console the relevant part of the blackboard associated with
            behaviours on the visited path
        display_activity_stream: print to the console a log of the activity on the blackboard
            over the last tick
    """
    def __init__(
        self,
        node: Node = None,
        display_only_visited_behaviours: bool = False,
        display_blackboard: bool = False,
        display_activity_stream: bool = False,
    ):
        super().__init__()
        self.node = node if node is not None else Node("tree_logger")
        self.display_only_visited_behaviours = display_only_visited_behaviours
        self.display_blackboard = display_blackboard
        self.display_activity_stream = display_activity_stream
        if self.display_activity_stream:
            blackboard.Blackboard.enable_activity_stream()

    def initialise(self) -> None:
        """Reset and initialise all variables."""
        self.root: typing.Optional[behaviour.Behaviour] = None
        super().initialise()
        if self.display_activity_stream:
            if blackboard.Blackboard.activity_stream is not None:
                blackboard.Blackboard.activity_stream.clear()

    def run(self, behaviour: behaviour.Behaviour) -> None:
        """
        Track the root of the tree and run :class:`~py_trees.visitors.SnapshotVisitor`.

        Args:
            behaviour: behaviour being visited.
        """
        if self.root is None:
            self.root = behaviour  # The first visited node is the root
        super().run(behaviour)
        
    def finalise(self) -> None:
        """Logs a summary on /rosout after all behaviours have been visited."""
        if self.root is not None:
            self.node.get_logger().info(
                "\n"
                + display.unicode_tree(
                    root=self.root,
                    show_only_visited=self.display_only_visited_behaviours,
                    show_status=False,
                    visited=self.visited,
                    previously_visited=self.previously_visited,
                )
            )
        if self.display_blackboard:
            self.node.get_logger().info(display.unicode_blackboard(key_filter=self.visited_blackboard_keys))
        if self.display_activity_stream:
            self.node.get_logger().info(display.unicode_blackboard_activity_stream())

# used for performance testing
# class StructuredMinimalSnapshotVisitor(py_trees.visitors.SnapshotVisitor):
#     def __init__(
#         self,
#         node: Node = None,
#         display_only_visited_behaviours: bool = False,
#         display_blackboard: bool = False,
#         display_activity_stream: bool = False,
#         debug: bool = False,
#     ):
#         super().__init__()
#         # Use provided ROS node or create a default one
#         self.node = node if node is not None else Node("minimal_tree_snapshot_publisher")

#         # Initialize Publishers
#         self.tree_pub = self.node.create_publisher(String, "~/minimal_tree_snapshot", 10)

#     def initialise(self) -> None:
#         self.root: typing.Optional[behaviour.Behaviour] = None
#         super().initialise()

#     def run(self, behaviour: behaviour.Behaviour) -> None:
#         self.root = behaviour
#         super().run(behaviour)

#     def finalise(self) -> None:
#         """Publishes the tree and blackboard data to ROS topics."""
#         # 4. Publish the Tree
#         tree_msg = String()
#         tree_msg.data = 'test2' # *200
#         self.tree_pub.publish(tree_msg)


class StructuredSnapshotVisitor(py_trees.visitors.SnapshotVisitor):
    """Profiles and serializes Behavior Tree execution metrics into standard ROS 2 string topics."""

    def __init__(
        self,
        node: typing.Optional[Node] = None,
        display_only_visited_behaviours: bool = False,
        display_blackboard: bool = False,
        display_activity_stream: bool = False,
        debug: bool = False,
    ):
        super().__init__()
        self.node = node if node is not None else Node("tree_snapshot_publisher")

        self.tree_pub = self.node.create_publisher(String, "~/tree_snapshot", 10)
        self.blackboard_pub = self.node.create_publisher(
            String, "~/blackboard_snapshot", 10
        )

        self.display_only_visited_behaviours = display_only_visited_behaviours
        self.display_blackboard = display_blackboard
        self.display_activity_stream = display_activity_stream
        self.debug = debug

        self.global_tick_count: int = 0
        self.tick_start_sim_times: typing.Dict[int, float] = {}

        self.node_start_times: typing.Dict[py_trees.common.Uuid, Time] = {}
        self.node_start_ticks: typing.Dict[py_trees.common.Uuid, int] = {}
        self.node_span_times: typing.Dict[py_trees.common.Uuid, float] = {}
        self.node_span_ticks: typing.Dict[py_trees.common.Uuid, int] = {}

        self.node_executed_intervals: typing.Dict[
            py_trees.common.Uuid, typing.List[typing.Tuple[int, int]]
        ] = collections.defaultdict(list)

        if self.display_activity_stream:
            blackboard.Blackboard.enable_activity_stream()

        self._modified_nodes: typing.List[
            typing.Tuple[behaviour.Behaviour, typing.Optional[str]]
        ] = []

    def initialise(self) -> None:
        self.root: typing.Optional[behaviour.Behaviour] = None
        super().initialise()

        self.global_tick_count += 1
        now = self.node.get_clock().now()
        self.tick_start_sim_times[self.global_tick_count] = now.nanoseconds / 1e9

        if self.display_activity_stream and blackboard.Blackboard.activity_stream is not None:
            blackboard.Blackboard.activity_stream.clear()

    @staticmethod
    def _merge_intervals(
        intervals: typing.List[typing.Tuple[int, int]],
    ) -> typing.List[typing.Tuple[int, int]]:
        if not intervals:
            return []

        sorted_intervals = sorted(intervals, key=lambda iv: iv[0])
        merged: typing.List[typing.Tuple[int, int]] = [sorted_intervals[0]]

        for start, end in sorted_intervals[1:]:
            last_start, last_end = merged[-1]
            if start <= last_end + 1:
                merged[-1] = (last_start, max(last_end, end))
            else:
                merged.append((start, end))

        return merged

    def _get_interval_duration(
        self, start_tick: int, end_tick: int, current_now_sec: float
    ) -> float:
        if start_tick not in self.tick_start_sim_times:
            return 0.0
        t_start = self.tick_start_sim_times[start_tick]
        if (end_tick + 1) in self.tick_start_sim_times:
            t_end = self.tick_start_sim_times[end_tick + 1]
        else:
            # Ongoing tick (end_tick == global_tick_count): determine tick duration dt
            if self.global_tick_count > 1 and 1 in self.tick_start_sim_times:
                avg_dt = (
                    self.tick_start_sim_times[self.global_tick_count]
                    - self.tick_start_sim_times[1]
                ) / (self.global_tick_count - 1)
            else:
                avg_dt = max(0.0, current_now_sec - t_start)

            if end_tick in self.tick_start_sim_times:
                t_end = max(current_now_sec, self.tick_start_sim_times[end_tick] + avg_dt)
            else:
                t_end = current_now_sec
        return max(0.0, t_end - t_start)

    def _find_retry_ancestor(
        self, node: behaviour.Behaviour
    ) -> typing.Optional[behaviour.Behaviour]:
        curr = node
        while curr is not None:
            if hasattr(curr, "num_failures") and hasattr(curr, "failures"):
                return curr
            curr = getattr(curr, "parent", None)
        return None

    def _get_retry_info(
        self, node: behaviour.Behaviour
    ) -> typing.Optional[typing.Tuple[int, int]]:
        retry_node = self._find_retry_ancestor(node)
        if retry_node is not None:
            current_retries = getattr(retry_node, "failures", 0)
            max_retries = getattr(retry_node, "num_failures", 0)
            return current_retries, max_retries
        return None

    def run(self, behaviour_node: behaviour.Behaviour) -> None:
        self.root = behaviour_node
        super().run(behaviour_node)

        node_id = behaviour_node.id
        node_intervals = self.node_executed_intervals[node_id]
        curr_tick = self.global_tick_count

        if node_intervals and node_intervals[-1][1] == curr_tick - 1:
            node_intervals[-1] = (node_intervals[-1][0], curr_tick)
        elif not node_intervals or node_intervals[-1][1] < curr_tick - 1:
            node_intervals.append((curr_tick, curr_tick))

        now = self.node.get_clock().now()
        if now.nanoseconds == 0:
            self.node_span_times[node_id] = 0.0
            self.node_span_ticks[node_id] = 0
            return

        if behaviour_node.status == py_trees.common.Status.RUNNING:
            if node_id not in self.node_start_times:
                self.node_start_times[node_id] = now
                self.node_start_ticks[node_id] = self.global_tick_count

            diff_ns = (now - self.node_start_times[node_id]).nanoseconds
            if diff_ns < 0:
                self.node_start_times[node_id] = now
                self.node_start_ticks[node_id] = self.global_tick_count
                span_time = 0.0
                span_ticks = 1
            else:
                span_time = diff_ns / 1e9
                span_ticks = max(1, self.global_tick_count - self.node_start_ticks[node_id] + 1)

        elif behaviour_node.status in (
            py_trees.common.Status.SUCCESS,
            py_trees.common.Status.FAILURE,
        ):
            if node_id in self.node_start_times:
                start_time = self.node_start_times.pop(node_id)
                start_tick = self.node_start_ticks.pop(node_id, self.global_tick_count)
                diff_ns = (now - start_time).nanoseconds
                span_time = diff_ns / 1e9 if diff_ns >= 0 else 0.0
                span_ticks = max(1, self.global_tick_count - start_tick + 1)
            else:
                span_time = self.node_span_times.get(node_id, 0.0)
                span_ticks = self.node_span_ticks.get(node_id, 1)
        else:  # INVALID / UNVISITED
            self.node_start_times.pop(node_id, None)
            self.node_start_ticks.pop(node_id, None)
            span_time = self.node_span_times.get(node_id, 0.0)
            span_ticks = self.node_span_ticks.get(node_id, 0)

        self.node_span_times[node_id] = span_time
        self.node_span_ticks[node_id] = span_ticks

    def _accumulate_times(
        self, node: behaviour.Behaviour
    ) -> typing.Tuple[int, float]:
        """Recursively calculates (total_ticks, total_seconds) such that:

        total = self_node_time + sum(immediate_child_totals).
        """
        now_sec = self.node.get_clock().now().nanoseconds / 1e9
        self_intervals = self.node_executed_intervals.get(node.id, [])

        # 1. Calculate self execution time and ticks
        if not node.children:
            # Leaf node: all its execution intervals constitute self work
            self_ticks = sum(end - start + 1 for start, end in self_intervals)
            self_time = sum(
                self._get_interval_duration(start, end, now_sec)
                for start, end in self_intervals
            )
        else:
            # Composite/Decorator node: self work is time spent when no children ran
            node_tick_set = {
                t
                for start, end in self_intervals
                for t in range(start, end + 1)
            }
            child_tick_set: typing.Set[int] = set()
            for child in node.children:
                c_intervals = self.node_executed_intervals.get(child.id, [])
                for start, end in c_intervals:
                    child_tick_set.update(range(start, end + 1))


            exclusive_ticks = sorted(node_tick_set - child_tick_set)
            self_ticks = len(exclusive_ticks)

            # Telescoping: merge contiguous exclusive ticks into intervals
            exclusive_intervals: typing.List[typing.Tuple[int, int]] = []
            for t in exclusive_ticks:
                if exclusive_intervals and exclusive_intervals[-1][1] == t - 1:
                    exclusive_intervals[-1] = (exclusive_intervals[-1][0], t)
                else:
                    exclusive_intervals.append((t, t))

            self_time = sum(
                self._get_interval_duration(start, end, now_sec)
                for start, end in exclusive_intervals
            )

        # 2. Accumulate totals from immediate children
        child_total_ticks = 0
        child_total_time = 0.0
        for child in node.children:
            c_ticks, c_time = self._accumulate_times(child)
            child_total_ticks += c_ticks
            child_total_time += c_time

        # 3. Compute strict additive totals
        total_ticks = self_ticks + child_total_ticks
        total_time = self_time + child_total_time

        # 4. Format as <total> (<self>) for parents, or pure <total> for leaf nodes
        if node.children:
            ticks_str = f"{total_ticks}t ({self_ticks}t)"
            time_str = f"{total_time:.2f}s ({self_time:.2f}s)"
        else:
            ticks_str = f"{total_ticks}t"
            time_str = f"{total_time:.2f}s"
        
        retry_info = self._get_retry_info(node)
        if retry_info is not None:
            curr_retries, max_retries = retry_info
            tag = f"[{ticks_str} / {time_str} across {curr_retries}/{max_retries} attempts]"
        else:
            tag = f"[{ticks_str} / {time_str}]"

        original_feedback = node.feedback_message
        self._modified_nodes.append((node, original_feedback))

        if original_feedback:
            node.feedback_message = f"{tag} {original_feedback}"
        else:
            node.feedback_message = tag

        return total_ticks, total_time

    def finalise(self) -> None:
        if self.root is not None:
            self._accumulate_times(self.root)

            tree_str = display.unicode_tree(
                root=self.root,
                show_only_visited=self.display_only_visited_behaviours,
                show_status=True,
                visited=self.visited,
                previously_visited=self.previously_visited,
            )

            for node, original_feedback in self._modified_nodes:
                node.feedback_message = original_feedback
            self._modified_nodes.clear()

            tree_msg = String()
            tree_msg.data = tree_str
            self.tree_pub.publish(tree_msg)

        if self.display_blackboard:
            bb_str = display.unicode_blackboard(key_filter=self.visited_blackboard_keys)
            bb_msg = String()
            bb_msg.data = bb_str
            self.blackboard_pub.publish(bb_msg)