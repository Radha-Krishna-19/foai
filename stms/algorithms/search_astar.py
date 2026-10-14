"""
Route search for vehicle agents: BFS, Uniform-Cost (Dijkstra), Greedy Best-First, A*, Weighted A*.

Owner: Member 3 (Vehicle Agent & Route Search)

Problem formulation (state-space search)
----------------------------------------
    State        : the intersection node the vehicle is at,  s in V
    Initial state: origin node
    Actions      : drive along any open outgoing road (u -> v)
    Transition   : RESULT(u, drive(u->v)) = v
    Goal test    : s == destination
    Step cost    : c(u, v) = BPR congested travel time of road (u -> v)   [seconds]
    Heuristic    : h(n) = d(n, goal) / v_max   (d = Euclidean or Manhattan metres,
                                               v_max = highest speed limit in the network)

Why h is admissible (never over-estimates):
    any route from n to goal is at least d(n, goal) metres long, and no road can be driven
    faster than v_max, so the true remaining time h*(n) >= d(n, goal) / v_max = h(n).
    BPR only *increases* travel time above free-flow, so congestion keeps h admissible.
Why h is consistent (monotone):
    h(u) <= c(u,v) + h(v)  follows from the triangle inequality d(u,g) <= d(u,v) + d(v,g)
    and c(u,v) >= len(u,v)/v_max >= d(u,v)/v_max.
    Consistency => f = g + h is non-decreasing along any path => the first time A* pops the
    goal it is optimal, and no node ever needs re-expansion.
Manhattan distance is admissible here only because every road in the grid is axis-aligned
(road length == Manhattan distance between its ends); on a general map use Euclidean.
"""

from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple
import heapq
import itertools

from stms.environment.road_network import RoadNetwork, RoadSegment


@dataclass
class SearchResult:
    algorithm: str
    path: List[str]
    cost: float
    nodes_expanded: int
    max_frontier: int
    trace: List[Dict] = field(default_factory=list)

    @property
    def found(self) -> bool:
        return bool(self.path)


def _cost_fn(edge_weight_func):
    if edge_weight_func is None:
        return lambda seg: seg.current_traversal_time()
    return edge_weight_func


def path_cost(network: RoadNetwork, path: List[str], edge_weight_func=None) -> float:
    w = _cost_fn(edge_weight_func)
    return sum(w(network.get_segment(a, b)) for a, b in zip(path, path[1:]))


def best_first_search(network: RoadNetwork, start: str, goal: str, heuristic: Callable[[str], float],
                      g_weight: float = 1.0, h_weight: float = 1.0, name: str = "A*",
                      edge_weight_func=None, record_trace: bool = False) -> SearchResult:
    """
    Generic best-first graph search, priority f(n) = g_weight * g(n) + h_weight * h(n).
        UCS            : g_weight=1, h_weight=0
        Greedy BFS     : g_weight=0, h_weight=1
        A*             : g_weight=1, h_weight=1
        Weighted A*    : g_weight=1, h_weight=w>1
    """
    w = _cost_fn(edge_weight_func)
    tie = itertools.count()
    h0 = heuristic(start)
    frontier: List[Tuple[float, int, str]] = [(h_weight * h0, next(tie), start)]
    g: Dict[str, float] = {start: 0.0}
    parent: Dict[str, Optional[str]] = {start: None}
    closed = set()
    expanded, max_frontier = 0, 1
    trace: List[Dict] = []

    while frontier:
        max_frontier = max(max_frontier, len(frontier))
        f, _, node = heapq.heappop(frontier)
        if node in closed:
            continue
        closed.add(node)
        expanded += 1
        if record_trace:
            best = {}
            for ff, _, n in frontier:
                if n not in closed and (n not in best or ff < best[n]):
                    best[n] = ff
            trace.append({"step": expanded, "expand": node, "g": round(g[node], 2),
                          "h": round(heuristic(node), 2), "f": round(f, 2),
                          "frontier": sorted([(round(ff, 1), n) for n, ff in best.items()])[:6],
                          "frontier_all": [{"node": n, "f": round(ff, 2), "g": round(g[n], 2), "h": round(heuristic(n), 2)}
                                           for n, ff in sorted(best.items(), key=lambda kv: kv[1])],
                          "closed": sorted(closed), "parent": dict(parent)})
        if node == goal:
            path = []
            cur = node
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            path.reverse()
            return SearchResult(name, path, g[node], expanded, max_frontier, trace)
        for seg in network.get_outgoing_segments(node):
            nb = seg.target
            if nb in closed or (edge_weight_func is None and seg.is_blocked):
                continue
            c = w(seg)
            if c == float("inf"):
                continue
            new_g = g[node] + c
            if nb not in g or new_g < g[nb]:
                g[nb] = new_g
                parent[nb] = node
                heapq.heappush(frontier, (g_weight * new_g + h_weight * heuristic(nb), next(tie), nb))
    return SearchResult(name, [], float("inf"), expanded, max_frontier, trace)


def breadth_first_search(network: RoadNetwork, start: str, goal: str, edge_weight_func=None,
                         record_trace: bool = False) -> SearchResult:
    """BFS: fewest road segments, ignores travel time (complete, optimal only for unit costs)."""
    w = _cost_fn(edge_weight_func)
    frontier = deque([start])
    parent: Dict[str, Optional[str]] = {start: None}
    expanded, max_frontier = 0, 1
    closed: List[str] = []
    trace: List[Dict] = []
    while frontier:
        max_frontier = max(max_frontier, len(frontier))
        node = frontier.popleft()
        expanded += 1
        closed.append(node)
        if record_trace:
            trace.append({"step": expanded, "expand": node, "g": None, "h": None, "f": None,
                          "frontier_all": [{"node": n} for n in frontier], "closed": sorted(closed),
                          "parent": dict(parent)})
        if node == goal:
            path = []
            cur = node
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            path.reverse()
            return SearchResult("BFS", path, path_cost(network, path, edge_weight_func), expanded, max_frontier, trace)
        for seg in network.get_outgoing_segments(node):
            if seg.target in parent or w(seg) == float("inf"):
                continue
            parent[seg.target] = node
            frontier.append(seg.target)
    return SearchResult("BFS", [], float("inf"), expanded, max_frontier, trace)


class AStarRouter:
    """
    Route planner used by every vehicle agent.
        algorithm : 'astar' | 'ucs' | 'greedy' | 'bfs' | 'weighted_astar'
        heuristic : 'euclidean' | 'manhattan' | 'zero'
    """

    def __init__(self, network: RoadNetwork, global_v_max: Optional[float] = None,
                 algorithm: str = "astar", heuristic: str = "euclidean", weight: float = 2.0):
        self.network = network
        self._v_max = global_v_max
        self.algorithm = algorithm
        self.heuristic_name = heuristic
        self.weight = weight
        self.total_searches = 0
        self.total_expanded = 0

    @property
    def global_v_max(self) -> float:
        return self._v_max if self._v_max is not None else self.network.max_speed_limit

    def heuristic(self, node: str, goal: str) -> float:
        if self.heuristic_name == "zero":
            return 0.0
        if self.heuristic_name == "manhattan":
            return self.network.get_manhattan_distance(node, goal) / self.global_v_max
        return self.network.get_euclidean_distance(node, goal) / self.global_v_max

    def search(self, start: str, goal: str, algorithm: Optional[str] = None,
               record_trace: bool = False, edge_weight_func=None) -> SearchResult:
        algo = algorithm or self.algorithm
        h = lambda n: self.heuristic(n, goal)
        if start == goal:
            return SearchResult(algo, [start], 0.0, 0, 0)
        if algo == "bfs":
            res = breadth_first_search(self.network, start, goal, edge_weight_func, record_trace)
        elif algo == "ucs":
            res = best_first_search(self.network, start, goal, h, 1.0, 0.0, "UCS (Dijkstra)", edge_weight_func, record_trace)
        elif algo == "greedy":
            res = best_first_search(self.network, start, goal, h, 0.0, 1.0, "Greedy Best-First", edge_weight_func, record_trace)
            if res.found:
                res.cost = path_cost(self.network, res.path, edge_weight_func)
        elif algo == "weighted_astar":
            res = best_first_search(self.network, start, goal, h, 1.0, self.weight, f"Weighted A* (w={self.weight:g})",
                                    edge_weight_func, record_trace)
        else:
            res = best_first_search(self.network, start, goal, h, 1.0, 1.0, "A*", edge_weight_func, record_trace)
        self.total_searches += 1
        self.total_expanded += res.nodes_expanded
        return res

    def find_optimal_path(self, start_node: str, goal_node: str,
                          edge_weight_func=None) -> Tuple[List[str], float, int]:
        """Backward-compatible API: (path, cost, nodes_expanded)."""
        r = self.search(start_node, goal_node, edge_weight_func=edge_weight_func)
        return r.path, r.cost, r.nodes_expanded
