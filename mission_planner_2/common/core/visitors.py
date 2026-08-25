import collections
import re
import typing

import py_trees
from py_trees import blackboard, behaviour, display
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
import json
from uuid import UUID

from mission_planner_interfaces.msg import (
    AttemptSnapshot as AttemptSnapshotMsg,
    BehaviourNodeSnapshot as BehaviourNodeSnapshotMsg,
    BehaviourTreeSnapshot as BehaviourTreeSnapshotMsg,
)

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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-


class StructuredSnapshotVisitor(py_trees.visitors.SnapshotVisitor):
    """Profiles and serializes Behavior Tree execution metrics into structured ROS 2 messages."""

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
        self.tree_pub = self.node.create_publisher(
            BehaviourTreeSnapshotMsg, "~/tree_snapshot", 10
        )

        self.display_only_visited_behaviours = display_only_visited_behaviours
        self.display_blackboard = display_blackboard
        self.display_activity_stream = display_activity_stream
        self.debug = debug

        self.global_tick_count: int = 0
        self.tick_start_sim_times: typing.Dict[int, float] = {}

        # Dictionaries storing (node_id: ([start ticks], [end ticks])) and (node_id: ([start times], [end times]))
        self.node_span_ticks: typing.Dict[
            UUID, typing.Tuple[typing.List[int], typing.List[int]]
        ] = collections.defaultdict(lambda: ([], []))
        self.node_span_times: typing.Dict[
            UUID, typing.Tuple[typing.List[float], typing.List[float]]
        ] = collections.defaultdict(lambda: ([], []))

        # Tracks raw execution intervals per node
        self.node_executed_intervals: typing.Dict[
            UUID, typing.List[typing.Tuple[int, int]]
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
            if curr_st <= prev_et:
                merged[-1] = (prev_st, max(prev_et, curr_et))
            elif curr_st == prev_et + 1:
                merged[-1] = (prev_st, curr_et)
            else:
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

    def _get_behaviour_symbol(self, node: behaviour.Behaviour) -> str:
        """Resolves visual display symbol matching py_trees conventions."""
        if isinstance(node, py_trees.composites.Sequence):
            return "{-}" if getattr(node, "memory", False) else "[-]"
        elif isinstance(node, py_trees.composites.Selector):
            return "{o}" if getattr(node, "memory", False) else "[o]"
        elif isinstance(node, py_trees.composites.Parallel):
            return "/_/"
        elif isinstance(node, py_trees.decorators.Decorator) or self._is_retry_node(node):
            return "-^-"
        elif isinstance(node, py_trees.behaviour.Behaviour):
            return "-->"
        return "-->"

    def _get_node_status_info(
        self, node: behaviour.Behaviour
    ) -> typing.Tuple[int, str, str]:
        """Returns (status_code, status_raw, status_str)."""
        is_visited = node.id in self.visited
        status = self.visited.get(node.id, getattr(node, "status", py_trees.common.Status.INVALID))

        if not is_visited and self.display_only_visited_behaviours:
            return (
                BehaviourNodeSnapshotMsg.STATUS_UNVISITED,
                "-",
                "unvisited",
            )

        if status == py_trees.common.Status.SUCCESS:
            return BehaviourNodeSnapshotMsg.STATUS_SUCCESS, "✓", "success"
        elif status == py_trees.common.Status.FAILURE:
            return BehaviourNodeSnapshotMsg.STATUS_FAILURE, "✕", "failure"
        elif status == py_trees.common.Status.RUNNING:
            return BehaviourNodeSnapshotMsg.STATUS_RUNNING, "*", "running"
        elif status == py_trees.common.Status.INVALID:
            return BehaviourNodeSnapshotMsg.STATUS_UNVISITED, "-", "unvisited"
        else:
            return BehaviourNodeSnapshotMsg.STATUS_INVALID, "-", "unknown"

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

        retry_ancestor = self._find_retry_ancestor(behaviour_node)
        attempt_idx = 0
        if retry_ancestor is not None:
            attempt_idx = getattr(retry_ancestor, "failures", 0)

        start_ticks_list, end_ticks_list = self.node_span_ticks[node_id]
        start_times_list, end_times_list = self.node_span_times[node_id]

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
        self,
        node: behaviour.Behaviour,
        metrics_map: typing.Dict[UUID, typing.Dict[str, typing.Any]],
    ) -> typing.Dict[str, typing.Any]:
        """Recursively calculates and caches metrics for all nodes in metrics_map."""
        now_sec = self.node.get_clock().now().nanoseconds / 1e9
        start_ticks, end_ticks = self.node_span_ticks[node.id]

        has_children = bool(getattr(node, "children", []))
        is_parallel = isinstance(node, py_trees.composites.Parallel)
        is_retry = self._is_retry_node(node)
        retry_ancestor = self._find_retry_ancestor(node)
        is_under_retry = retry_ancestor is not None
        retry_info = self._get_retry_info(node)

        # 1. Process child nodes recursively, ensuring their metrics are stored in metrics_map
        child_metrics: typing.List[typing.Dict[str, typing.Any]] = []
        if has_children:
            child_metrics = [
                self._accumulate_times(child, metrics_map)
                for child in node.children
            ]

        attempts_data: typing.Dict[int, typing.Dict[str, typing.Any]] = {}

        if is_under_retry:
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
                cum_sub_t, cum_sub_s = 0, 0.0
                cum_self_t, cum_self_s = 0, 0.0
                cum_st, cum_et = st, et
            else:
                c_sts = [cm["cum_st"] for cm in child_metrics if cm["cum_st"] > 0]
                c_ets = [cm["cum_et"] for cm in child_metrics if cm["cum_et"] > 0]
                if st == 0 and c_sts:
                    st = min(c_sts)
                    et = max(c_ets)
                cum_st, cum_et = st, et

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

        # 2. Format Metric String and Tag
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

        node_metrics = {
            "cum_span_t": cum_span_t,
            "cum_span_s": cum_span_s,
            "cum_self_t": cum_self_t,
            "cum_self_s": cum_self_s,
            "cum_sub_t": cum_sub_t,
            "cum_sub_s": cum_sub_s,
            "cum_st": cum_st,
            "cum_et": cum_et,
            "cum_intervals": cum_intervals,
            "cum_t_str": cum_t_str,
            "cum_s_str": cum_s_str,
            "tag": tag,
            "clean_feedback": clean_feedback,
            "attempts": attempts_data,
        }

        # Crucial: Store node metrics for every node in the tree
        metrics_map[node.id] = node_metrics
        return node_metrics

    def _build_paths_recursive(
        self,
        node: behaviour.Behaviour,
        parent_path: typing.Optional[str],
        sibling_count_map: typing.Dict[str, int],
        node_paths: typing.Dict[UUID, str],
        node_depths: typing.Dict[UUID, int],
        depth: int = 0,
    ) -> None:
        sibling_key = f"{parent_path or 'root'}/{node.name}"
        sibling_count = sibling_count_map[sibling_key]
        sibling_count_map[sibling_key] += 1
        current_path = f"{parent_path}/{node.name}" if parent_path else f"/{node.name}"
        if sibling_count > 0:
            current_path += f"#{sibling_count}"

        node_paths[node.id] = current_path
        node_depths[node.id] = depth
        for child in getattr(node, "children", []):
            self._build_paths_recursive(
                child, current_path, sibling_count_map, node_paths, node_depths, depth + 1
            )

    def finalise(self) -> None:
        """Called after all nodes have finished their tick."""
        if self.root is not None:
            curr = self.root
            while getattr(curr, "parent", None) is not None:
                curr = curr.parent
            self.root = curr

            # 1. Accumulate metrics for all nodes into metrics_dict
            metrics_dict: typing.Dict[UUID, typing.Dict[str, typing.Any]] = {}
            self._accumulate_times(self.root, metrics_dict)

            # 2. Render unicode text tree for logs and backward compatibility
            tree_str = display.unicode_tree(
                root=self.root,
                show_only_visited=self.display_only_visited_behaviours,
                show_status=True,
                visited=self.visited,
                previously_visited=self.previously_visited,
            )

            # 3. Restore original feedback messages
            for node, original_feedback in self._modified_nodes:
                node.feedback_message = original_feedback
            self._modified_nodes.clear()

            # 4. Build hierarchical paths and depths
            sibling_count_map: typing.Dict[str, int] = collections.defaultdict(int)
            node_paths: typing.Dict[UUID, str] = {}
            node_depths: typing.Dict[UUID, int] = {}
            self._build_paths_recursive(
                self.root, None, sibling_count_map, node_paths, node_depths, 0
            )

            # 5. Populate structured ROS 2 message
            tree_msg = BehaviourTreeSnapshotMsg()
            tree_msg.header.stamp = self.node.get_clock().now().to_msg()
            tree_msg.header.frame_id = ""
            tree_msg.global_tick_count = self.global_tick_count
            tree_msg.raw_text_tree = tree_str

            node_msgs: typing.List[BehaviourNodeSnapshotMsg] = []
            for behaviour_node in self.root.iterate():
                node_id = behaviour_node.id
                metrics = metrics_dict.get(node_id, {})
                status_code, status_raw, status_str = self._get_node_status_info(behaviour_node)
                symbol = self._get_behaviour_symbol(behaviour_node)
                depth = node_depths.get(node_id, 0)
                current_path = node_paths.get(node_id, f"/{behaviour_node.name}")

                parent = getattr(behaviour_node, "parent", None)
                parent_path = node_paths.get(parent.id, "") if parent is not None else ""
                child_paths = [
                    node_paths.get(c.id, "") for c in getattr(behaviour_node, "children", [])
                ]

                has_children = bool(getattr(behaviour_node, "children", []))
                retry_info = self._get_retry_info(behaviour_node)

                node_msg = BehaviourNodeSnapshotMsg()
                node_msg.id = current_path
                node_msg.name = behaviour_node.name
                node_msg.behaviour_type = type(behaviour_node).__name__
                node_msg.symbol = symbol
                node_msg.depth = depth
                node_msg.parent_id = parent_path
                node_msg.child_ids = child_paths
                node_msg.has_children = has_children
                node_msg.is_active = (node_id in self.visited)
                node_msg.status = status_code
                node_msg.status_raw = status_raw
                node_msg.status_str = status_str

                if retry_info is not None:
                    curr_retries, max_retries = retry_info
                    node_msg.attempts = curr_retries
                    node_msg.max_attempts = max_retries
                    node_msg.retry_str = f"{curr_retries}/{max_retries}"
                else:
                    node_msg.attempts = -1
                    node_msg.max_attempts = -1
                    node_msg.retry_str = ""

                cum_st = metrics.get("cum_st", 0)
                cum_et = metrics.get("cum_et", 0)
                cum_span_t = metrics.get("cum_span_t", 0)
                cum_t_str = metrics.get("cum_t_str", "0t (0t 0t)" if has_children else "0t")
                cum_s_str = metrics.get("cum_s_str", "0.00s (0.00s 0.00s)" if has_children else "0.00s")

                node_msg.ticks = cum_t_str
                node_msg.seconds = cum_s_str
                node_msg.ticks_num = cum_span_t
                node_msg.seconds_num = float(metrics.get("cum_span_s", 0.0))
                node_msg.self_ticks = metrics.get("cum_self_t", 0)
                node_msg.self_seconds = float(metrics.get("cum_self_s", 0.0))
                node_msg.sub_ticks = metrics.get("cum_sub_t", 0)
                node_msg.sub_seconds = float(metrics.get("cum_sub_s", 0.0))
                node_msg.start_tick = cum_st
                node_msg.end_tick = cum_et
                node_msg.is_same_tick = (cum_st > 0 and cum_st == cum_et and cum_span_t == 1)

                clean_fb = metrics.get("clean_feedback", "")
                node_msg.feedback = clean_fb

                tag = metrics.get("tag", f"[{cum_t_str} / {cum_s_str}]")
                indent = "    " * depth
                dash = f" -- {tag}" if tag else ""
                fb_str = f" {clean_fb}" if clean_fb else ""
                node_msg.raw_line = f"{indent}{symbol} {behaviour_node.name} [{status_raw}]{dash}{fb_str}"

                # Populate attempt history
                attempt_msgs: typing.List[AttemptSnapshotMsg] = []
                for att_idx, att in sorted(metrics.get("attempts", {}).items(), key=lambda x: x[0]):
                    att_msg = AttemptSnapshotMsg()
                    att_msg.attempt_index = att_idx
                    att_msg.ticks = att["t_str"]
                    att_msg.seconds = att["s_str"]
                    att_msg.ticks_num = att["span_t"]
                    att_msg.seconds_num = float(att["span_s"])
                    att_msg.self_ticks = att["self_t"]
                    att_msg.self_seconds = float(att["self_s"])
                    att_msg.sub_ticks = att["sub_t"]
                    att_msg.sub_seconds = float(att["sub_s"])
                    att_msg.start_tick = att["st"]
                    att_msg.end_tick = att["et"]
                    att_msg.is_same_tick = (att["st"] > 0 and att["st"] == att["et"] and att["span_t"] == 1)
                    att_msg.feedback = ""
                    att_msg.status = status_code
                    att_msg.status_str = status_str
                    attempt_msgs.append(att_msg)

                node_msg.attempt_history = attempt_msgs
                node_msgs.append(node_msg)

            tree_msg.nodes = node_msgs
            self.tree_pub.publish(tree_msg)