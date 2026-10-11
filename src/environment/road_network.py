"""
Road network: intersections are graph nodes, directed road segments are edges.
Edge cost = BPR (Bureau of Public Roads) congested travel time.

Owner: Member 1 (Environment & Simulation)
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import math
import networkx as nx

BLOCKED_COST = float("inf")


@dataclass
class RoadSegment:
    """
    Directed single-lane road segment  source -> target.

    Queue discipline: `vehicles` is FIFO; index 0 is the vehicle closest to the stop line.
    Saturation flow: at most one vehicle crosses the stop line every `discharge_headway` seconds
    (2.0 s  ==  1800 veh/h/lane, the standard HCM saturation flow).
    """
    segment_id: str
    source: str
    target: str
    length_meters: float
    speed_limit_mps: float = 13.88          # 50 km/h
    capacity: int = 20                      # vehicles that physically fit (~7.5 m per car)
    is_blocked: bool = False
    discharge_headway: float = 2.0
    vehicles: List[str] = field(default_factory=list)
    last_discharge_time: float = -1e9
    total_discharged: int = 0
    incident_speed_mps: Optional[float] = None   # set when an accident reduces speed (lane closed)

    @property
    def effective_speed(self) -> float:
        return self.incident_speed_mps if self.incident_speed_mps is not None else self.speed_limit_mps

    @property
    def vehicle_count(self) -> int:
        return len(self.vehicles)

    @property
    def density(self) -> float:
        return min(1.0, len(self.vehicles) / max(1, self.capacity))

    @property
    def free_flow_time(self) -> float:
        return self.length_meters / max(0.1, self.speed_limit_mps)

    def traversal_time_at(self, speed_mps: float) -> float:
        """BPR travel time if the road's free speed were `speed_mps`."""
        v_c = self.vehicle_count / max(1, self.capacity)
        return self.length_meters / max(0.1, speed_mps) * (1.0 + 0.15 * v_c ** 4)

    def current_traversal_time(self) -> float:
        """BPR:  t = t0 * (1 + alpha * (V/C)^beta),  alpha = 0.15, beta = 4.  Infinite if blocked."""
        if self.is_blocked:
            return BLOCKED_COST
        return self.traversal_time_at(self.effective_speed)

    def can_accept(self) -> bool:
        return (not self.is_blocked) and len(self.vehicles) < self.capacity

    def add_vehicle(self, vehicle_id: str) -> bool:
        if not self.can_accept():
            return False
        if vehicle_id not in self.vehicles:
            self.vehicles.append(vehicle_id)
        return True

    def remove_vehicle(self, vehicle_id: str):
        if vehicle_id in self.vehicles:
            self.vehicles.remove(vehicle_id)

    def can_discharge(self, now: float) -> bool:
        return now - self.last_discharge_time >= self.discharge_headway

    def mark_discharge(self, now: float):
        self.last_discharge_time = now
        self.total_discharged += 1


class RoadNetwork:
    """Directed graph G = (V, E) of intersections and road segments."""

    def __init__(self):
        self.graph = nx.DiGraph()
        self.node_positions: Dict[str, Tuple[float, float]] = {}
        self.segments: Dict[Tuple[str, str], RoadSegment] = {}
        self._by_id: Dict[str, RoadSegment] = {}

    def add_intersection(self, node_id: str, x: float, y: float):
        self.graph.add_node(node_id, pos=(x, y))
        self.node_positions[node_id] = (x, y)

    def add_road(self, source: str, target: str, length_meters: Optional[float] = None,
                 speed_limit_mps: float = 13.88, capacity: int = 20) -> RoadSegment:
        if length_meters is None:
            length_meters = self.get_euclidean_distance(source, target)
        seg = RoadSegment(segment_id=f"{source}->{target}", source=source, target=target,
                          length_meters=length_meters, speed_limit_mps=speed_limit_mps, capacity=capacity)
        self.segments[(source, target)] = seg
        self._by_id[seg.segment_id] = seg
        self.graph.add_edge(source, target, segment=seg, weight=seg.free_flow_time)
        return seg

    def get_segment(self, source: str, target: str) -> Optional[RoadSegment]:
        return self.segments.get((source, target))

    def segment_by_id(self, seg_id: str) -> Optional[RoadSegment]:
        return self._by_id.get(seg_id)

    def get_incoming_segments(self, node: str) -> List[RoadSegment]:
        return [self.segments[(u, node)] for u in self.graph.predecessors(node)]

    def get_outgoing_segments(self, node: str) -> List[RoadSegment]:
        return [self.segments[(node, v)] for v in self.graph.successors(node)]

    def set_incident(self, source: str, target: str, is_blocked: bool):
        seg = self.get_segment(source, target)
        if seg:
            seg.is_blocked = is_blocked
            self.graph[source][target]["weight"] = BLOCKED_COST if is_blocked else seg.free_flow_time

    def set_slowdown(self, source: str, target: str, speed_mps: Optional[float]):
        """Partial incident (e.g. one lane closed): vehicles can pass but only at `speed_mps`."""
        seg = self.get_segment(source, target)
        if seg:
            seg.incident_speed_mps = speed_mps

    def get_euclidean_distance(self, a: str, b: str) -> float:
        (x1, y1), (x2, y2) = self.node_positions[a], self.node_positions[b]
        return math.hypot(x2 - x1, y2 - y1)

    def get_manhattan_distance(self, a: str, b: str) -> float:
        (x1, y1), (x2, y2) = self.node_positions[a], self.node_positions[b]
        return abs(x2 - x1) + abs(y2 - y1)

    @property
    def max_speed_limit(self) -> float:
        return max((s.speed_limit_mps for s in self.segments.values()), default=13.88)

    def position_on_segment(self, seg: RoadSegment, dist: float) -> Tuple[float, float]:
        (x1, y1), (x2, y2) = self.node_positions[seg.source], self.node_positions[seg.target]
        f = max(0.0, min(1.0, dist / max(1e-6, seg.length_meters)))
        return x1 + (x2 - x1) * f, y1 + (y2 - y1) * f

    def get_active_vehicles_count(self) -> int:
        return sum(len(seg.vehicles) for seg in self.segments.values())
