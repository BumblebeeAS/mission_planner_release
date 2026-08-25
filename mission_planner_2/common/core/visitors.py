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
#         self.tree_pub.publish(tree_msg)import collections4




class StructuredSnapshotVisitor(py_trees.visitors.SnapshotVisitor):
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

        # Multi-attempt tracking keyed by attempt path tuple (e.g. (0,), (0, 2), (1, 1))
        self.node_span_ticks: typing.Dict[
            UUID, typing.Dict[typing.Tuple[int, ...], typing.Tuple[int, int]]
        ] = collections.defaultdict(dict)
        self.node_span_times: typing.Dict[
            UUID, typing.Dict[typing.Tuple[int, ...], typing.Tuple[float, float]]
        ] = collections.defaultdict(dict)

        # Tracks raw execution intervals per attempt path
        self.node_executed_intervals: typing.Dict[
            UUID, typing.Dict[typing.Tuple[int, ...], typing.List[typing.Tuple[int, int]]]
        ] = collections.defaultdict(lambda: collections.defaultdict(list))

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
        if start_tick not in self.tick_start_sim_times or start_tick <= 0:
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
        """Checks if a node is strictly a retry decorator."""
        return (
            (hasattr(node, "num_failures") and hasattr(node, "failures"))
            or (
                hasattr(py_trees, "decorators")
                and hasattr(py_trees.decorators, "Retry")
                and isinstance(node, py_trees.decorators.Retry)
            )
        )

    def _find_all_retry_ancestors(
        self, node: behaviour.Behaviour
    ) -> typing.List[behaviour.Behaviour]:
        """Finds all retry decorator ancestors strictly above this behaviour, ordered outer to inner."""
        ancestors = []
        curr = getattr(node, "parent", None)
        while curr is not None:
            if self._is_retry_node(curr):
                ancestors.append(curr)
            curr = getattr(curr, "parent", None)
        ancestors.reverse()
        return ancestors

    def _get_retry_info(
        self, node: behaviour.Behaviour
    ) -> typing.Optional[typing.Tuple[int, int]]:
        """Extracts retry failure count and maximum retry count ONLY for retry nodes."""
        if self._is_retry_node(node):
            current_retries = getattr(node, "failures", 0)
            max_retries = getattr(node, "num_failures", 0)
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

        retry_ancestors = self._find_all_retry_ancestors(behaviour_node)
        ancestor_path = tuple(getattr(a, "failures", 0) for a in retry_ancestors)
        # If the node itself is a retry node, its own attempt path includes its current attempt
        if self._is_retry_node(behaviour_node):
            attempt_path = ancestor_path + (getattr(behaviour_node, "failures", 0),)
        else:
            attempt_path = ancestor_path

        # Update executed intervals for this attempt path
        node_intervals = self.node_executed_intervals[node_id][attempt_path]
        if node_intervals and node_intervals[-1][1] == curr_tick - 1:
            node_intervals[-1] = (node_intervals[-1][0], curr_tick)
        elif not node_intervals or node_intervals[-1][1] < curr_tick - 1:
            node_intervals.append((curr_tick, curr_tick))

        st, et = self.node_span_ticks[node_id].get(attempt_path, (0, 0))
        st_time, et_time = self.node_span_times[node_id].get(attempt_path, (0.0, 0.0))

        if st == 0:
            st = curr_tick
            st_time = now_sec
        et = curr_tick
        et_time = now_sec

        self.node_span_ticks[node_id][attempt_path] = (st, et)
        self.node_span_times[node_id][attempt_path] = (st_time, et_time)

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
        """Recursively calculates metrics for multi-dimensional retry paths."""
        now_sec = self.node.get_clock().now().nanoseconds / 1e9
        has_children = bool(getattr(node, "children", []))
        is_parallel = isinstance(node, py_trees.composites.Parallel)
        is_retry = self._is_retry_node(node)
        retry_ancestors = self._find_all_retry_ancestors(node)
        is_under_retry = len(retry_ancestors) > 0
        retry_info = self._get_retry_info(node)

        # 1. Recurse down children first
        child_metrics: typing.List[typing.Dict[str, typing.Any]] = []
        if has_children:
            child_metrics = [
                self._accumulate_times(child, metrics_map)
                for child in node.children
            ]

        expected_depth = len(retry_ancestors) + (1 if is_retry else 0)

        # 2. Discover only actively executed attempt paths
        attempt_paths_set: typing.Set[typing.Tuple[int, ...]] = set()

        for p, (st_val, _) in self.node_span_ticks[node.id].items():
            if len(p) == expected_depth and st_val > 0:
                attempt_paths_set.add(p)

        # Include paths from children (truncated to this node's attempt depth)
        for cm in child_metrics:
            for c_path in cm["attempts"].keys():
                if len(c_path) >= expected_depth:
                    attempt_paths_set.add(c_path[:expected_depth])

        # If currently visited in this tick, ensure current attempt path is captured
        if node.id in self.visited:
            curr_prefix = tuple(getattr(a, "failures", 0) for a in retry_ancestors)
            curr_path = curr_prefix + (getattr(node, "failures", 0),) if is_retry else curr_prefix
            attempt_paths_set.add(curr_path)

        attempts_data: typing.Dict[typing.Tuple[int, ...], typing.Dict[str, typing.Any]] = {}

        for att_path in sorted(attempt_paths_set):
            st, et = self.node_span_ticks[node.id].get(att_path, (0, 0))
            own_ivs = self.node_executed_intervals[node.id].get(att_path, [])

            if not has_children:
                if st > 0:
                    merged_ivs = self.merge_intervals(own_ivs) if own_ivs else [(st, et)]
                    att_span_t = sum(iv_et - iv_st + 1 for iv_st, iv_et in merged_ivs)
                    att_span_s = sum(
                        self._get_interval_duration(iv_st, iv_et, now_sec)
                        for iv_st, iv_et in merged_ivs
                    )
                    att_intervals = merged_ivs
                else:
                    att_span_t = 0
                    att_span_s = 0.0
                    att_intervals = []
                att_sub_t, att_sub_s = 0, 0.0
                att_self_t, att_self_s = att_span_t, att_span_s
            else:
                child_contributions = []
                for cm in child_metrics:
                    matching_c = [
                        c_att
                        for c_key, c_att in cm["attempts"].items()
                        if c_key[:len(att_path)] == att_path
                    ]
                    if matching_c:
                        c_st_vals = [c["st"] for c in matching_c if c["st"] > 0]
                        c_et_vals = [c["et"] for c in matching_c if c["et"] > 0]
                        c_st = min(c_st_vals) if c_st_vals else 0
                        c_et = max(c_et_vals) if c_et_vals else 0
                        c_ivs: typing.List[typing.Tuple[int, int]] = []
                        for c in matching_c:
                            c_ivs.extend(c.get("intervals", []))
                        merged_c_ivs = self.merge_intervals(c_ivs)
                        c_span_t = sum(iv_et - iv_st + 1 for iv_st, iv_et in merged_c_ivs)
                        c_span_s = sum(
                            self._get_interval_duration(iv_st, iv_et, now_sec)
                            for iv_st, iv_et in merged_c_ivs
                        )
                        child_contributions.append({
                            "st": c_st,
                            "et": c_et,
                            "span_t": c_span_t,
                            "span_s": c_span_s,
                            "intervals": merged_c_ivs,
                        })

                c_sts = [c["st"] for c in child_contributions if c["st"] > 0]
                c_ets = [c["et"] for c in child_contributions if c["et"] > 0]

                if st == 0 and c_sts:
                    st = min(c_sts)
                    et = max(c_ets)
                elif st > 0 and c_sts:
                    st = min(st, min(c_sts))
                    et = max(et, max(c_ets))

                if is_parallel:
                    att_sub_t = max((c["span_t"] for c in child_contributions), default=0)
                    att_sub_s = max((c["span_s"] for c in child_contributions), default=0.0)
                    att_intervals = [(st, et)] if st > 0 else []
                    own_t = (et - st + 1) if st > 0 else 0
                    own_s = self._get_interval_duration(st, et, now_sec) if st > 0 else 0.0
                    att_span_t = max(own_t, att_sub_t)
                    att_span_s = max(own_s, att_sub_s)
                    att_self_t = max(0, att_span_t - att_sub_t)
                    att_self_s = max(0.0, att_span_s - att_sub_s)
                else:
                    child_ivs: typing.List[typing.Tuple[int, int]] = []
                    for c in child_contributions:
                        child_ivs.extend(c.get("intervals", []))
                    merged_child_ivs = self.merge_intervals(child_ivs)
                    att_sub_t = sum(iv_et - iv_st + 1 for iv_st, iv_et in merged_child_ivs)
                    att_sub_s = sum(
                        self._get_interval_duration(iv_st, iv_et, now_sec)
                        for iv_st, iv_et in merged_child_ivs
                    )
                    if own_ivs:
                        merged_all_ivs = self.merge_intervals(own_ivs + child_ivs)
                        att_span_t = sum(iv_et - iv_st + 1 for iv_st, iv_et in merged_all_ivs)
                        att_span_s = sum(
                            self._get_interval_duration(iv_st, iv_et, now_sec)
                            for iv_st, iv_et in merged_all_ivs
                        )
                        att_intervals = merged_all_ivs
                    else:
                        att_span_t = att_sub_t
                        att_span_s = att_sub_s
                        att_intervals = merged_child_ivs

                    att_self_t = max(0, att_span_t - att_sub_t)
                    att_self_s = max(0.0, att_span_s - att_sub_s)

            # Filter out attempt paths that never triggered
            if st == 0 and att_span_t == 0:
                continue

            t_str, s_str = self._format_metric_str(
                att_span_t, att_self_t, att_sub_t, att_span_s, att_self_s, att_sub_s, has_children
            )

            attempts_data[att_path] = {
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

        # Calculate cumulative metrics across all executed attempts (Total = Exact Sum of Attempts)
        if attempts_data:
            cum_span_t = sum(a["span_t"] for a in attempts_data.values())
            cum_span_s = sum(a["span_s"] for a in attempts_data.values())
            cum_self_t = sum(a["self_t"] for a in attempts_data.values())
            cum_self_s = sum(a["self_s"] for a in attempts_data.values())
            cum_sub_t = sum(a["sub_t"] for a in attempts_data.values())
            cum_sub_s = sum(a["sub_s"] for a in attempts_data.values())

            all_ivs = []
            for a in attempts_data.values():
                all_ivs.extend(a.get("intervals", []))
            cum_intervals = self.merge_intervals(all_ivs)

            valid_sts = [a["st"] for a in attempts_data.values() if a["st"] > 0]
            valid_ets = [a["et"] for a in attempts_data.values() if a["et"] > 0]
            cum_st = min(valid_sts) if valid_sts else 0
            cum_et = max(valid_ets) if valid_ets else 0
        else:
            st, et = self.node_span_ticks[node.id].get((), (0, 0))
            cum_st, cum_et = st, et
            cum_span_t = (et - st + 1) if st > 0 else 0
            cum_span_s = self._get_interval_duration(st, et, now_sec) if st > 0 else 0.0
            cum_sub_t, cum_sub_s = 0, 0.0
            cum_self_t, cum_self_s = cum_span_t, cum_span_s
            cum_intervals = [(st, et)] if st > 0 else []

        cum_t_str, cum_s_str = self._format_metric_str(
            cum_span_t, cum_self_t, cum_sub_t, cum_span_s, cum_self_s, cum_sub_s, has_children
        )

        cum_range = f" @{cum_st}-{cum_et}" if cum_st > 0 else ""
        has_multiple_attempts = len(attempts_data) > 0 and (is_under_retry or is_retry)

        if has_multiple_attempts:
            tag_parts = [f"{cum_t_str} / {cum_s_str}{cum_range}"]
            for att_path, att in sorted(attempts_data.items(), key=lambda x: x[0]):
                att_range = f" @{att['st']}-{att['et']}" if att["st"] > 0 else ""
                if len(att_path) == 1:
                    prefix = f"x{att_path[0]}"
                elif len(att_path) > 1:
                    prefix = f"x{','.join(map(str, att_path))}"
                else:
                    prefix = "x0"
                tag_parts.append(f"{prefix}: {att['t_str']} / {att['s_str']}{att_range}")

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
        node.feedback_message = f"{tag} {clean_feedback}".strip() if clean_feedback else tag

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

            metrics_dict: typing.Dict[UUID, typing.Dict[str, typing.Any]] = {}
            self._accumulate_times(self.root, metrics_dict)

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

            sibling_count_map: typing.Dict[str, int] = collections.defaultdict(int)
            node_paths: typing.Dict[UUID, str] = {}
            node_depths: typing.Dict[UUID, int] = {}
            self._build_paths_recursive(
                self.root, None, sibling_count_map, node_paths, node_depths, 0
            )

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

                # Populate only actually executed attempts
                attempt_msgs: typing.List[AttemptSnapshotMsg] = []
                for att_path, att in sorted(metrics.get("attempts", {}).items(), key=lambda x: x[0]):
                    att_msg = AttemptSnapshotMsg()
                    att_msg.attempt_index = att_path[-1] if len(att_path) > 0 else 0
                    att_msg.attempt_indices = list(att_path)
                    att_msg.attempt_key = ",".join(map(str, att_path))
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