"""
Vehicle Agent - goal-based commuter that plans with A* and re-plans when the world changes.

Owner: Member 3 (Vehicle Agent & Route Search)

reroute_policy:
    'broadcast' : re-plan on TMC INCIDENT_ALERT / CONGESTION_WARNING and on local discovery (default)
    'local'     : re-plan only when it physically reaches a blocked road (no V2I communication)
    'static'    : never re-plan (legacy fixed-route navigation) - used as a baseline

Knowledge model (partial observability): a vehicle plans on its OWN belief about the roads.
    known_blocked : roads it believes are closed
    known_speed   : roads it believes are slowed by an incident
Beliefs change only through (a) TMC broadcasts or (b) its own sensors when it reaches the road.
"""

from typing import Any, Dict, List, Optional
from src.agents.base_agent import BaseAgent, AgentType
from src.algorithms.search_astar import AStarRouter
from src.environment.road_network import RoadNetwork, RoadSegment
from src.protocols.message import MessageBroker, Performative


class VehicleAgent(BaseAgent):
    CAR_GAP = 7.5            # metres occupied per queued car
    REPLAN_GAIN = 0.85       # congestion re-plan only if new route is >= 15 % faster

    def __init__(self, agent_id: str, broker: MessageBroker, network: RoadNetwork,
                 origin_node: str, destination_node: str, router: AStarRouter,
                 departure_time: float = 0.0, reroute_policy: str = "broadcast"):
        super().__init__(agent_id, AgentType.VEHICLE, broker)
        self.network = network
        self.origin_node = origin_node
        self.destination_node = destination_node
        self.router = router
        self.departure_time = departure_time
        self.reroute_policy = reroute_policy

        self.current_node = origin_node
        self.current_segment: Optional[RoadSegment] = None
        self.distance_on_segment = 0.0
        self.current_speed = 0.0
        self.max_speed = 13.88
        self.acceleration = 2.5
        self.yields_to_queue = True          # emergency vehicles set False (traffic pulls aside)
        self.obeys_headway = True
        self.obeys_speed_limit = True

        self.planned_path: List[str] = []
        self.path_index = 0
        self.has_arrived = False
        self.departed = False
        self.arrival_time: Optional[float] = None

        self.total_travel_time = 0.0
        self.total_wait_time = 0.0
        self.stops = 0
        self._was_moving = False
        self.fuel_consumed_liters = 0.0
        self.plan_count = 0
        self.reroute_count = 0
        self.nodes_expanded_total = 0
        self.known_blocked = set()
        self.known_speed = {}

        self._plan_from(self.current_node, prefix=[])

    # ---- planning -----------------------------------------------------
    def _plan_from(self, node: str, prefix: List[str]) -> bool:
        if node == self.destination_node:
            self.planned_path = prefix + [node]
            self.path_index = 0
            return True
        res = self.router.search(node, self.destination_node, edge_weight_func=self.edge_weight)
        self.nodes_expanded_total += res.nodes_expanded
        self.plan_count += 1
        if res.found:
            self.planned_path = prefix + res.path
            self.path_index = 0
            return True
        return False

    def edge_weight(self, seg: RoadSegment) -> float:
        """Travel time of a road according to THIS vehicle's beliefs (BPR with believed speed)."""
        if seg.segment_id in self.known_blocked:
            return float("inf")
        return seg.traversal_time_at(self.known_speed.get(seg.segment_id, seg.speed_limit_mps))

    def learn(self, advisory: dict) -> bool:
        """Update beliefs from an incident advisory. Returns True if anything changed."""
        status = advisory.get("status")
        seg = advisory.get("blocked_segment") or advisory.get("cleared_segment") or advisory.get("segment")
        if not seg:
            return False
        sid = f"{seg[0]}->{seg[1]}"
        if status == "ROAD_BLOCKED":
            self.known_blocked.add(sid)
        elif status == "ROAD_SLOWED":
            self.known_speed[sid] = float(advisory.get("speed_mps", 3.0))
        elif status == "ROAD_CLEARED":
            self.known_blocked.discard(sid)
            self.known_speed.pop(sid, None)
        else:
            return False
        return True

    def replan_route(self) -> bool:
        """Re-plan from the next decision point. If on a road u->v, the car must still reach v."""
        self.reroute_count += 1
        if self.current_segment is not None:
            return self._plan_from(self.current_segment.target, prefix=[self.current_segment.source])
        return self._plan_from(self.current_node, prefix=[])

    def remaining_route_cost(self) -> float:
        start = self.path_index + (1 if self.current_segment is not None else 0)
        nodes = self.planned_path[start:]
        return sum(self.edge_weight(self.network.get_segment(a, b)) for a, b in zip(nodes, nodes[1:]))

    def is_segment_on_route(self, seg_tuple) -> bool:
        if not seg_tuple or len(self.planned_path) < 2:
            return False
        u, v = seg_tuple
        start = self.path_index + (1 if self.current_segment is not None else 0)
        return any(self.planned_path[i] == u and self.planned_path[i + 1] == v
                   for i in range(start, len(self.planned_path) - 1))

    # ---- SENSE --------------------------------------------------------
    def perceive(self, env: Dict[str, Any]):
        for msg in self.inbox:
            if self.reroute_policy != "broadcast":
                continue
            kind = msg.content.get("type")
            if kind == "INCIDENT_ALERT" and self.learn(msg.content):
                seg = msg.content.get("blocked_segment") or msg.content.get("segment")
                if msg.content.get("status") == "ROAD_CLEARED" or self.is_segment_on_route(tuple(seg) if seg else None):
                    self.replan_route()
            elif kind == "CONGESTION_WARNING" and self.departed:
                self._maybe_congestion_replan()
        self.inbox.clear()

    def _maybe_congestion_replan(self):
        if self.current_segment is None:
            return
        old_cost = self.remaining_route_cost()
        nxt = self.current_segment.target
        if nxt == self.destination_node:
            return
        res = self.router.search(nxt, self.destination_node, edge_weight_func=self.edge_weight)
        if res.found and res.cost < self.REPLAN_GAIN * old_cost:
            self.planned_path = [self.current_segment.source] + res.path
            self.path_index = 0
            self.reroute_count += 1

    # ---- REASON -------------------------------------------------------
    def reason(self) -> float:
        """Target speed for this tick (simple acceleration model)."""
        if self.has_arrived:
            return 0.0
        return min(self.max_speed, self.current_speed + self.acceleration)

    # ---- helpers ------------------------------------------------------
    def _next_segment(self) -> Optional[RoadSegment]:
        if self.path_index + 2 >= len(self.planned_path):
            return None
        return self.network.get_segment(self.planned_path[self.path_index + 1], self.planned_path[self.path_index + 2])

    def _signal_green(self, seg: RoadSegment, env) -> bool:
        agent = env.get("intersections", {}).get(seg.target)
        if agent is None:
            return True
        return agent.is_green_for(seg.segment_id)

    def xy(self):
        if self.current_segment is None:
            return self.network.node_positions[self.current_node]
        return self.network.position_on_segment(self.current_segment, self.distance_on_segment)

    def on_crossed(self, node: str, env):
        """Hook for subclasses (emergency / bus release their priority here)."""

    def on_approach(self, env):
        """Hook for subclasses (emergency / bus request priority here)."""

    # ---- ACT ----------------------------------------------------------
    def act(self, env: Dict[str, Any]):
        if self.has_arrived:
            return
        now = env.get("time", 0.0)
        self.total_travel_time += 1.0

        if self.current_segment is None:
            if self.path_index >= len(self.planned_path) - 1:
                self._arrive(now)
                return
            seg = self.network.get_segment(self.planned_path[self.path_index], self.planned_path[self.path_index + 1])
            if seg is not None and seg.is_blocked and self.reroute_policy != "static":
                self.known_blocked.add(seg.segment_id)          # seen with own eyes
                self.replan_route()
                seg = self.network.get_segment(self.planned_path[self.path_index], self.planned_path[self.path_index + 1])
            if seg is None or not seg.add_vehicle(self.agent_id):
                self._idle()
                return
            self.current_segment = seg
            self.distance_on_segment = 0.0

        self.on_approach(env)
        seg = self.current_segment
        at_exit = seg.target == self.destination_node and self.path_index + 2 >= len(self.planned_path)

        # can we cross the stop line this tick?
        nxt = None if at_exit else self._next_segment()
        if nxt is not None and nxt.is_blocked and self.reroute_policy != "static":
            self.known_blocked.add(nxt.segment_id)              # seen at the junction
            self.replan_route()
            nxt = self._next_segment()
        if seg.incident_speed_mps is not None:
            self.known_speed[seg.segment_id] = seg.incident_speed_mps   # learnt too late: already on it
        downstream_ok = at_exit or (nxt is not None and nxt.can_accept())
        headway_ok = (not self.obeys_headway) or seg.target not in env.get("intersections", {}) or seg.can_discharge(now)
        may_cross = self._signal_green(seg, env) and downstream_ok and headway_ok

        boundary = float("inf") if may_cross else seg.length_meters
        if self.yields_to_queue:
            idx = seg.vehicles.index(self.agent_id) if self.agent_id in seg.vehicles else 0
            if idx > 0:
                leader = env.get("vehicle_map", {}).get(seg.vehicles[idx - 1])
                if leader is not None and leader.current_segment is seg:
                    boundary = min(boundary, max(0.0, leader.distance_on_segment - self.CAR_GAP))

        speed = self.reason()
        cap = seg.incident_speed_mps if seg.incident_speed_mps is not None else float("inf")
        if self.obeys_speed_limit:
            cap = min(cap, seg.speed_limit_mps)
        speed = min(speed, cap)
        new_pos = min(self.distance_on_segment + speed, boundary)
        advance = max(0.0, new_pos - self.distance_on_segment)
        self.distance_on_segment += advance
        self.current_speed = advance
        if advance < 0.5:
            self._idle()
        else:
            self._was_moving = True
            self._fuel(advance)

        if self.distance_on_segment >= seg.length_meters and may_cross:
            seg.remove_vehicle(self.agent_id)
            if seg.target in env.get("intersections", {}):
                seg.mark_discharge(now)
            self.path_index += 1
            self.current_node = seg.target
            self.current_segment = None
            self.on_crossed(seg.target, env)
            if self.path_index >= len(self.planned_path) - 1:
                self._arrive(now)
                return
            nseg = self._next_segment_from_index()
            if nseg is not None and nseg.add_vehicle(self.agent_id):
                self.current_segment = nseg
                self.distance_on_segment = max(0.0, min(self.distance_on_segment - seg.length_meters, 10.0))

    def _next_segment_from_index(self) -> Optional[RoadSegment]:
        if self.path_index + 1 >= len(self.planned_path):
            return None
        return self.network.get_segment(self.planned_path[self.path_index], self.planned_path[self.path_index + 1])

    def _idle(self):
        self.total_wait_time += 1.0
        if self._was_moving:
            self.stops += 1
        self._was_moving = False
        self.current_speed = 0.0
        self.fuel_consumed_liters += 0.00035          # idle burn ~0.35 ml/s

    def _fuel(self, metres: float):
        speed_kmh = metres * 3.6
        rate = (0.07 / 1000.0) * (1.0 + 0.005 * abs(speed_kmh - 50))   # L per metre
        self.fuel_consumed_liters += rate * metres

    def _arrive(self, now: float):
        self.has_arrived = True
        self.arrival_time = now
        self.current_segment = None
        self.current_node = self.destination_node

    def step(self, env: Dict[str, Any]):
        if self.has_arrived:
            return
        if env.get("time", 0.0) < self.departure_time:
            # not on the road yet, but its navigation app still listens (pre-trip re-planning)
            if self.broker.peek(self.agent_id):
                self.sim_time = env.get("time", self.sim_time)
                self.receive_messages()
                self.perceive(env)
            return
        self.departed = True
        super().step(env)
