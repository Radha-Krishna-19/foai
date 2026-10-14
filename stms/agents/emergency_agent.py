"""
Emergency Vehicle Agent (ambulance / fire engine).

Owner: Member 4 (Ambulance Agent)

Protocol (per intersection on the route):
    when ETA to the intersection <= preempt_horizon  ->  PREEMPT_OVERRIDE (request green on my axis)
    intersection replies AGREE, clears the conflicting phase through YELLOW, holds my green
    after the ambulance crosses                      ->  CANCEL (intersection resumes Max-Pressure)
Traffic already queued on the ambulance's road is assumed to pull aside (yields_to_queue = False),
so the only thing that can stop it is a red signal - exactly what preemption removes.
"""

from typing import Any, Dict, List, Set
from stms.agents.vehicle_agent import VehicleAgent
from stms.agents.base_agent import AgentType
from stms.algorithms.search_astar import AStarRouter
from stms.environment.road_network import RoadNetwork
from stms.protocols.message import MessageBroker, Performative


def travel_axis(network: RoadNetwork, a: str, b: str) -> str:
    (x1, y1), (x2, y2) = network.node_positions[a], network.node_positions[b]
    return "NS" if abs(y2 - y1) >= abs(x2 - x1) else "EW"


class PriorityVehicleMixin:
    """Shared look-ahead logic: which signalised intersections are coming up, and how soon."""

    def upcoming_intersections(self, env) -> List[tuple]:
        """Returns [(node, axis, eta_seconds)] for signalised nodes ahead on the route."""
        out = []
        if self.current_segment is None:
            return out
        inters = env.get("intersections", {})
        dist = self.current_segment.length_meters - self.distance_on_segment
        idx = self.path_index + 1
        prev = self.planned_path[self.path_index]
        while idx < len(self.planned_path):
            node = self.planned_path[idx]
            if node in inters:
                out.append((node, travel_axis(self.network, prev, node), dist / self.max_speed))
            if idx + 1 < len(self.planned_path):
                seg = self.network.get_segment(node, self.planned_path[idx + 1])
                dist += seg.length_meters if seg else 0.0
            prev = node
            idx += 1
        return out


class EmergencyVehicleAgent(PriorityVehicleMixin, VehicleAgent):
    def __init__(self, agent_id: str, broker: MessageBroker, network: RoadNetwork,
                 origin_node: str, destination_node: str, router: AStarRouter,
                 departure_time: float = 0.0, lookahead_intersections: int = 2,
                 preemption_enabled: bool = True, preempt_horizon: float = 15.0):
        super().__init__(agent_id, broker, network, origin_node, destination_node, router, departure_time)
        self.agent_type = AgentType.EMERGENCY
        self.max_speed = 16.67                    # 60 km/h
        self.acceleration = 3.5
        self.yields_to_queue = False
        self.obeys_headway = False
        self.obeys_speed_limit = False
        self.lookahead_intersections = lookahead_intersections
        self.preemption_enabled = preemption_enabled
        self.preempt_horizon = preempt_horizon
        self.preempted_intersections: List[str] = []
        self.granted: Set[str] = set()
        self._active_requests: Set[str] = set()

    def perceive(self, env: Dict[str, Any]):
        for msg in self.inbox:
            if msg.performative == Performative.AGREE and msg.ontology == "emergency-preemption":
                self.granted.add(msg.sender_id)
        super().perceive(env)

    def on_approach(self, env):
        if not self.preemption_enabled:
            return
        for node, axis, eta in self.upcoming_intersections(env)[: self.lookahead_intersections]:
            if eta <= self.preempt_horizon and node not in self._active_requests:
                self.send_message(node, Performative.PREEMPT_OVERRIDE,
                                  {"emergency_vehicle_id": self.agent_id, "corridor_direction": axis,
                                   "eta_s": round(eta, 1), "urgency": "CRITICAL"},
                                  ontology="emergency-preemption", protocol="fipa-request")
                self._active_requests.add(node)
                if node not in self.preempted_intersections:
                    self.preempted_intersections.append(node)

    def on_crossed(self, node: str, env):
        if node in self._active_requests:
            self.send_message(node, Performative.CANCEL, {"emergency_vehicle_id": self.agent_id,
                                                          "status": "CLEARED_INTERSECTION"},
                              ontology="emergency-preemption", protocol="fipa-request")
            self._active_requests.discard(node)
