import collections
import re
import typing

import py_trees
from py_trees import blackboard, behaviour, display
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
import json

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
#         self.tree_pub.publish(tree_msg)import collections


class StructuredSnapshotVisitor(py_trees.visitors.SnapshotVisitor):
    """Profiles and serializes Behavior Tree execution metrics into standard ROS 2 string topics."""

    def __init__(
        self,
        node: typing.Optional[Node] = None,
        display_only_visited_behaviours: bool = False,
        display_blackboard: bool = False,
        display_activity_stream: bool = False,
        debug: bool = True,
    ):
        super().__init__()
        self.node = node if node is not None else Node("tree_snapshot_publisher")
        self.tree_pub = self.node.create_publisher(String, "~/tree_snapshot", 10)

        self.display_only_visited_behaviours = display_only_visited_behaviours
        self.display_blackboard = display_blackboard
        self.display_activity_stream = display_activity_stream
        self.debug = debug

        self.global_tick_count: int = 0
        self.tick_start_sim_times: typing.Dict[int, float] = {}

        # Dictionaries storing (node_id: ([start ticks], [end ticks])) and (node_id: ([start times], [end times]))
        self.node_span_ticks: typing.Dict[
            py_trees.common.Uuid, typing.Tuple[typing.List[int], typing.List[int]]
        ] = collections.defaultdict(lambda: ([], []))
        self.node_span_times: typing.Dict[
            py_trees.common.Uuid, typing.Tuple[typing.List[float], typing.List[float]]
        ] = collections.defaultdict(lambda: ([], []))

        # Tracks raw execution intervals per node
        self.node_executed_intervals: typing.Dict[
            py_trees.common.Uuid, typing.List[typing.Tuple[int, int]]
        ] = collections.defaultdict(list)

        self._modified_nodes: typing.List[
            typing.Tuple[behaviour.Behaviour, typing.Optional[str]]
        ] = []

    def initialise(self) -> None:
        """Called once before each tick of the tree begins."""
        self.root: typing.Optional[behaviour.Behaviour] = None
        super().initialise()
        self.global_tick_count += 1
        now = self.node.get_clock().now()
        self.tick_start_sim_times[self.global_tick_count] = now.nanoseconds / 1e9

    def _get_interval_duration(
        self, start_tick: int, end_tick: int, current_now_sec: float
    ) -> float:
        """Find ROS clock sim time elapsed between 2 tick values."""
        if start_tick not in self.tick_start_sim_times or start_tick == 0:
            return 0.0
        t_start = self.tick_start_sim_times[start_tick]
        if (end_tick + 1) in self.tick_start_sim_times:
            t_end = self.tick_start_sim_times[end_tick + 1]
        else:
            t_end = current_now_sec
        return max(0.0, t_end - t_start)

    def merge_intervals(
        self, intervals: typing.List[typing.Tuple[int, int]]
    ) -> typing.List[typing.Tuple[int, int]]:
        """Merges overlapping or contiguous [start_tick, end_tick] intervals."""
        valid = [iv for iv in intervals if iv[0] > 0 and iv[1] >= iv[0]]
        if not valid:
            return []
        sorted_ivs = sorted(valid, key=lambda x: (x[0], x[1]))
        merged = [sorted_ivs[0]]
        for curr_st, curr_et in sorted_ivs[1:]:
            prev_st, prev_et = merged[-1]
            if curr_st <= prev_et:  # Overlapping or same tick
                merged[-1] = (prev_st, max(prev_et, curr_et))
            elif curr_st == prev_et + 1:  # Contiguous ticks
                merged[-1] = (prev_st, curr_et)
            else:  # Disjoint gap
                merged.append((curr_st, curr_et))
        return merged

    def _is_retry_node(self, node: behaviour.Behaviour) -> bool:
        """Checks if a node is a retry decorator."""
        return (
            (hasattr(node, "num_failures") and hasattr(node, "failures"))
            or (
                hasattr(py_trees, "decorators")
                and hasattr(py_trees.decorators, "Retry")
                and isinstance(node, py_trees.decorators.Retry)
            )
        )

    def _find_retry_ancestor(
        self, node: behaviour.Behaviour
    ) -> typing.Optional[behaviour.Behaviour]:
        """Finds the nearest retry decorator ancestor for this behaviour."""
        curr = getattr(node, "parent", None)
        while curr is not None:
            if self._is_retry_node(curr):
                return curr
            curr = getattr(curr, "parent", None)
        return None

    def _get_retry_info(
        self, node: behaviour.Behaviour
    ) -> typing.Optional[typing.Tuple[int, int]]:
        """Extracts current retry failure count and maximum retry count."""
        target_node = node if self._is_retry_node(node) else self._find_retry_ancestor(node)
        if target_node is not None:
            current_retries = getattr(target_node, "failures", 0)
            max_retries = getattr(target_node, "num_failures", 0)
            return current_retries, max_retries
        return None

    def run(self, behaviour_node: behaviour.Behaviour) -> None:
        """Executes once per visited behavior node in this tick."""
        if self.root is None:
            curr = behaviour_node
            while getattr(curr, "parent", None) is not None:
                curr = curr.parent
            self.root = curr

        super().run(behaviour_node)
        node_id = behaviour_node.id
        curr_tick = self.global_tick_count
        now_sec = self.node.get_clock().now().nanoseconds / 1e9

        # Update executed intervals
        node_intervals = self.node_executed_intervals[node_id]
        if node_intervals and node_intervals[-1][1] == curr_tick - 1:
            node_intervals[-1] = (node_intervals[-1][0], curr_tick)
        elif not node_intervals or node_intervals[-1][1] < curr_tick - 1:
            node_intervals.append((curr_tick, curr_tick))

        # Determine attempt index: Retry node itself stays in attempt 0, child nodes index by retry count
        retry_ancestor = self._find_retry_ancestor(behaviour_node)
        attempt_idx = 0
        if retry_ancestor is not None:
            attempt_idx = getattr(retry_ancestor, "failures", 0)

        # Record span ticks and span time entries
        start_ticks_list, end_ticks_list = self.node_span_ticks[node_id]
        start_times_list, end_times_list = self.node_span_times[node_id]

        # Expand retry rows if new retry triggered
        while len(start_ticks_list) <= attempt_idx:
            start_ticks_list.append(0)
            end_ticks_list.append(0)
            start_times_list.append(0.0)
            end_times_list.append(0.0)

        if start_ticks_list[attempt_idx] == 0:
            start_ticks_list[attempt_idx] = curr_tick
            start_times_list[attempt_idx] = now_sec

        end_ticks_list[attempt_idx] = curr_tick
        end_times_list[attempt_idx] = now_sec

    @staticmethod
    def _format_metric_str(
        span_t: int,
        self_t: int,
        sub_t: int,
        span_s: float,
        self_s: float,
        sub_s: float,
        has_children: bool,
    ) -> typing.Tuple[str, str]:
        if has_children:
            ticks_str = f"{span_t}t ({self_t}t {sub_t}t)"
            time_str = f"{span_s:.2f}s ({self_s:.2f}s {sub_s:.2f}s)"
        else:
            ticks_str = f"{span_t}t"
            time_str = f"{span_s:.2f}s"
        return ticks_str, time_str

    def _accumulate_times(
        self, node: behaviour.Behaviour
    ) -> typing.Dict[str, typing.Any]:
        """
        Recursively calculates span, subtree, and self ticks/time cumulatively and per-retry attempt.
        """
        now_sec = self.node.get_clock().now().nanoseconds / 1e9
        start_ticks, end_ticks = self.node_span_ticks[node.id]

        has_children = bool(node.children)
        is_parallel = isinstance(node, py_trees.composites.Parallel)
        is_retry = self._is_retry_node(node)
        retry_ancestor = self._find_retry_ancestor(node)
        is_under_retry = retry_ancestor is not None
        retry_info = self._get_retry_info(node)

        # 1. Process Child Nodes Recursively
        child_metrics: typing.List[typing.Dict[str, typing.Any]] = []
        if has_children:
            child_metrics = [self._accumulate_times(child) for child in node.children]

        attempts_data: typing.Dict[int, typing.Dict[str, typing.Any]] = {}

        if is_under_retry:
            # Case 1: Node is inside a Retry decorator
            curr_retries = getattr(retry_ancestor, "failures", 0)
            num_attempts = max(len(start_ticks), curr_retries + 1)
            for cm in child_metrics:
                num_attempts = max(num_attempts, len(cm["attempts"]))
            if num_attempts == 0:
                num_attempts = 1

            for att_idx in range(num_attempts):
                st = start_ticks[att_idx] if att_idx < len(start_ticks) else 0
                et = end_ticks[att_idx] if att_idx < len(end_ticks) else 0

                if not has_children:
                    if st > 0:
                        att_span_t = et - st + 1
                        att_span_s = self._get_interval_duration(st, et, now_sec)
                        att_intervals = [(st, et)]
                    else:
                        att_span_t = 0
                        att_span_s = 0.0
                        att_intervals = []
                    att_sub_t, att_sub_s = 0, 0.0
                    att_self_t, att_self_s = 0, 0.0
                else:
                    c_atts = [
                        cm["attempts"][att_idx]
                        for cm in child_metrics
                        if att_idx in cm["attempts"]
                    ]
                    c_sts = [c["st"] for c in c_atts if c["st"] > 0]
                    c_ets = [c["et"] for c in c_atts if c["et"] > 0]

                    if st == 0 and c_sts:
                        st = min(c_sts)
                        et = max(c_ets)

                    if is_parallel:
                        att_sub_t = max((c["span_t"] for c in c_atts), default=0)
                        att_sub_s = max((c["span_s"] for c in c_atts), default=0.0)
                        att_intervals = [(st, et)] if st > 0 else []
                    else:
                        child_ivs: typing.List[typing.Tuple[int, int]] = []
                        for c in c_atts:
                            if c.get("intervals"):
                                child_ivs.extend(c["intervals"])
                            elif c["st"] > 0 and c["et"] >= c["st"]:
                                child_ivs.append((c["st"], c["et"]))

                        merged_child_ivs = self.merge_intervals(child_ivs)
                        att_sub_t = sum(iv_et - iv_st + 1 for iv_st, iv_et in merged_child_ivs)
                        att_sub_s = sum(
                            self._get_interval_duration(iv_st, iv_et, now_sec)
                            for iv_st, iv_et in merged_child_ivs
                        )
                        att_intervals = [(st, et)] if st > 0 else merged_child_ivs

                    own_t = (et - st + 1) if st > 0 else 0
                    own_s = self._get_interval_duration(st, et, now_sec) if st > 0 else 0.0

                    att_span_t = max(own_t, att_sub_t)
                    att_span_s = max(own_s, att_sub_s)
                    att_self_t = max(0, att_span_t - att_sub_t)
                    att_self_s = max(0.0, att_span_s - att_sub_s)

                t_str, s_str = self._format_metric_str(
                    att_span_t, att_self_t, att_sub_t, att_span_s, att_self_s, att_sub_s, has_children
                )

                attempts_data[att_idx] = {
                    "span_t": att_span_t,
                    "span_s": att_span_s,
                    "self_t": att_self_t,
                    "self_s": att_self_s,
                    "sub_t": att_sub_t,
                    "sub_s": att_sub_s,
                    "st": st,
                    "et": et,
                    "intervals": att_intervals,
                    "t_str": t_str,
                    "s_str": s_str,
                }

            cum_span_t = sum(a["span_t"] for a in attempts_data.values())
            cum_span_s = sum(a["span_s"] for a in attempts_data.values())
            cum_self_t = sum(a["self_t"] for a in attempts_data.values())
            cum_self_s = sum(a["self_s"] for a in attempts_data.values())
            cum_sub_t = sum(a["sub_t"] for a in attempts_data.values())
            cum_sub_s = sum(a["sub_s"] for a in attempts_data.values())

            valid_sts = [a["st"] for a in attempts_data.values() if a["st"] > 0]
            valid_ets = [a["et"] for a in attempts_data.values() if a["et"] > 0]
            cum_st = min(valid_sts) if valid_sts else 0
            cum_et = max(valid_ets) if valid_ets else 0

            cum_intervals: typing.List[typing.Tuple[int, int]] = []
            for a in attempts_data.values():
                cum_intervals.extend(a.get("intervals", []))
            cum_intervals = self.merge_intervals(cum_intervals)

        elif is_retry:
            # Case 2: Node IS a Retry decorator
            curr_retries = getattr(node, "failures", 0)
            num_attempts = curr_retries + 1
            for cm in child_metrics:
                num_attempts = max(num_attempts, len(cm["attempts"]))
            if num_attempts == 0:
                num_attempts = 1

            for att_idx in range(num_attempts):
                c_atts = [
                    cm["attempts"][att_idx]
                    for cm in child_metrics
                    if att_idx in cm["attempts"]
                ]
                c_sts = [c["st"] for c in c_atts if c["st"] > 0]
                c_ets = [c["et"] for c in c_atts if c["et"] > 0]

                att_st = min(c_sts) if c_sts else 0
                att_et = max(c_ets) if c_ets else 0

                child_ivs = []
                for c in c_atts:
                    if c.get("intervals"):
                        child_ivs.extend(c["intervals"])
                    elif c["st"] > 0 and c["et"] >= c["st"]:
                        child_ivs.append((c["st"], c["et"]))

                merged_child_ivs = self.merge_intervals(child_ivs)
                att_sub_t = sum(iv_et - iv_st + 1 for iv_st, iv_et in merged_child_ivs)
                att_sub_s = sum(
                    self._get_interval_duration(iv_st, iv_et, now_sec)
                    for iv_st, iv_et in merged_child_ivs
                )

                own_att_t = (att_et - att_st + 1) if att_st > 0 else 0
                own_att_s = self._get_interval_duration(att_st, att_et, now_sec) if att_st > 0 else 0.0

                att_span_t = max(own_att_t, att_sub_t)
                att_span_s = max(own_att_s, att_sub_s)
                att_self_t = max(0, att_span_t - att_sub_t)
                att_self_s = max(0.0, att_span_s - att_sub_s)
                att_intervals = [(att_st, att_et)] if att_st > 0 else merged_child_ivs

                t_str, s_str = self._format_metric_str(
                    att_span_t, att_self_t, att_sub_t, att_span_s, att_self_s, att_sub_s, True
                )

                attempts_data[att_idx] = {
                    "span_t": att_span_t,
                    "span_s": att_span_s,
                    "self_t": att_self_t,
                    "self_s": att_self_s,
                    "sub_t": att_sub_t,
                    "sub_s": att_sub_s,
                    "st": att_st,
                    "et": att_et,
                    "intervals": att_intervals,
                    "t_str": t_str,
                    "s_str": s_str,
                }

            # Retry node cumulative span does not reset
            st = start_ticks[0] if (start_ticks and start_ticks[0] > 0) else 0
            et = end_ticks[0] if (end_ticks and end_ticks[0] > 0) else 0
            if st == 0:
                valid_sts = [a["st"] for a in attempts_data.values() if a["st"] > 0]
                valid_ets = [a["et"] for a in attempts_data.values() if a["et"] > 0]
                st = min(valid_sts) if valid_sts else 0
                et = max(valid_ets) if valid_ets else 0

            cum_st = st
            cum_et = et

            own_t = (et - st + 1) if st > 0 else 0
            own_s = self._get_interval_duration(st, et, now_sec) if st > 0 else 0.0

            cum_sub_t = sum(a["sub_t"] for a in attempts_data.values())
            cum_sub_s = sum(a["sub_s"] for a in attempts_data.values())

            cum_span_t = max(own_t, cum_sub_t)
            cum_span_s = max(own_s, cum_sub_s)
            cum_self_t = max(0, cum_span_t - cum_sub_t)
            cum_self_s = max(0.0, cum_span_s - cum_sub_s)
            cum_intervals = [(cum_st, cum_et)] if cum_st > 0 else []

        else:
            # Case 3: Node is NOT under retry and NOT a retry node (e.g. Root, Sequence, Leaf outside retry)
            st = start_ticks[0] if (start_ticks and start_ticks[0] > 0) else 0
            et = end_ticks[0] if (end_ticks and end_ticks[0] > 0) else 0

            if not has_children:
                if st > 0:
                    cum_span_t = et - st + 1
                    cum_span_s = self._get_interval_duration(st, et, now_sec)
                    cum_intervals = [(st, et)]
                else:
                    cum_span_t = 0
                    cum_span_s = 0.0
                    cum_intervals = []
                cum_sub_t = 0
                cum_sub_s = 0.0
                cum_self_t = 0
                cum_self_s = 0.0
                cum_st = st
                cum_et = et
            else:
                c_sts = [cm["cum_st"] for cm in child_metrics if cm["cum_st"] > 0]
                c_ets = [cm["cum_et"] for cm in child_metrics if cm["cum_et"] > 0]
                if st == 0 and c_sts:
                    st = min(c_sts)
                    et = max(c_ets)
                cum_st = st
                cum_et = et

                if is_parallel:
                    cum_sub_t = max((cm["cum_span_t"] for cm in child_metrics), default=0)
                    cum_sub_s = max((cm["cum_span_s"] for cm in child_metrics), default=0.0)
                    cum_intervals = [(st, et)] if st > 0 else []
                else:
                    child_ivs = []
                    for cm in child_metrics:
                        if cm.get("cum_intervals"):
                            child_ivs.extend(cm["cum_intervals"])
                        elif cm["cum_st"] > 0 and cm["cum_et"] >= cm["cum_st"]:
                            child_ivs.append((cm["cum_st"], cm["cum_et"]))
                    merged_child_ivs = self.merge_intervals(child_ivs)
                    cum_sub_t = sum(iv_et - iv_st + 1 for iv_st, iv_et in merged_child_ivs)
                    cum_sub_s = sum(
                        self._get_interval_duration(iv_st, iv_et, now_sec)
                        for iv_st, iv_et in merged_child_ivs
                    )
                    cum_intervals = [(st, et)] if st > 0 else merged_child_ivs

                own_t = (et - st + 1) if st > 0 else 0
                own_s = self._get_interval_duration(st, et, now_sec) if st > 0 else 0.0

                cum_span_t = max(own_t, cum_sub_t)
                cum_span_s = max(own_s, cum_sub_s)
                cum_self_t = max(0, cum_span_t - cum_sub_t)
                cum_self_s = max(0.0, cum_span_s - cum_sub_s)

        # 2. Format Display String and Tag
        cum_t_str, cum_s_str = self._format_metric_str(
            cum_span_t, cum_self_t, cum_sub_t, cum_span_s, cum_self_s, cum_sub_s, has_children
        )

        cum_range = f" @{cum_st}-{cum_et}" if cum_st > 0 else ""
        has_multiple_attempts = len(attempts_data) > 0 and (is_under_retry or is_retry)

        if has_multiple_attempts:
            tag_parts = [f"{cum_t_str} / {cum_s_str}{cum_range}"]
            for att_idx in sorted(attempts_data.keys()):
                att = attempts_data[att_idx]
                att_range = f" @{att['st']}-{att['et']}" if att["st"] > 0 else ""
                tag_parts.append(
                    f"x{att_idx}: {att['t_str']} / {att['s_str']}{att_range}"
                )
            if is_retry and retry_info is not None:
                curr_retries, max_retries = retry_info
                tag_parts.append(f"across {curr_retries}/{max_retries} attempts")
            tag = f"[{' | '.join(tag_parts)}]"
        else:
            tag = f"[{cum_t_str} / {cum_s_str}{cum_range}]"

        original_feedback = node.feedback_message
        clean_feedback = ""
        if original_feedback:
            clean_feedback = re.sub(r"\[[^\]]*\]", "", original_feedback)
            clean_feedback = re.sub(
                r"across\s+\d+\s*(?:/|\s+of\s+)\s*\d+\s+attempts", "", clean_feedback, flags=re.IGNORECASE
            )
            clean_feedback = re.sub(
                r"retries:\s*\d+(?:\s*/\s*\d+)?", "", clean_feedback, flags=re.IGNORECASE
            )
            clean_feedback = re.sub(
                r"status:\s*\d+\s+failure\s+from\s+\d+", "", clean_feedback, flags=re.IGNORECASE
            )
            clean_feedback = re.sub(r"\s+", " ", clean_feedback).strip()

        self._modified_nodes.append((node, original_feedback))

        if clean_feedback:
            node.feedback_message = f"{tag} {clean_feedback}"
        else:
            node.feedback_message = tag

        return {
            "cum_span_t": cum_span_t,
            "cum_span_s": cum_span_s,
            "cum_self_t": cum_self_t,
            "cum_self_s": cum_self_s,
            "cum_sub_t": cum_sub_t,
            "cum_sub_s": cum_sub_s,
            "cum_st": cum_st,
            "cum_et": cum_et,
            "cum_intervals": cum_intervals,
            "attempts": attempts_data,
        }

    def finalise(self) -> None:
        """Called after all nodes have finished their tick."""
        if self.root is not None:
            curr = self.root
            while getattr(curr, "parent", None) is not None:
                curr = curr.parent
            self.root = curr

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