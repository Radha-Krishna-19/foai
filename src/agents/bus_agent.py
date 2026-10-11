"""
Bus Agent - public-transport vehicle that NEGOTIATES Transit Signal Priority (TSP).

Owner: Member 4 (Priority Vehicles: Emergency + Bus)

Unlike the ambulance (absolute override), a bus only PROPOSES priority. The intersection is free to
REJECT_PROPOSAL when giving the bus green would hurt a long cross-street queue. This is a
genuine two-sided negotiation (FIPA propose / accept-proposal / reject-proposal):

    Bus  --PROPOSE{axis, eta, passengers}-->  Intersection
    Intersection evaluates cross-street queue  ->  ACCEPT_PROPOSAL | REJECT_PROPOSAL (with reason)
    Bus  --CANCEL-->  Intersection             after crossing (releases the priority)
"""

from typing import Any, Dict, List, Set
from src.agents.vehicle_agent import VehicleAgent
from src.agents.emergency_agent import PriorityVehicleMixin
from src.agents.base_agent import AgentType
from src.protocols.message import Performative


class BusAgent(PriorityVehicleMixin, VehicleAgent):
    def __init__(self, agent_id, broker, network, origin_node, destination_node, router,
                 departure_time: float = 0.0, tsp_enabled: bool = True, passengers: int = 40,
                 propose_horizon: float = 12.0):
        super().__init__(agent_id, broker, network, origin_node, destination_node, router, departure_time)
        self.agent_type = AgentType.BUS
        self.max_speed = 11.11             # 40 km/h
        self.acceleration = 1.5
        self.tsp_enabled = tsp_enabled
        self.passengers = passengers
        self.propose_horizon = propose_horizon
        self._proposed: Set[str] = set()
        self.accepted: List[str] = []
        self.rejected: List[str] = []

    def perceive(self, env: Dict[str, Any]):
        for msg in self.inbox:
            if msg.ontology != "transit-signal-priority":
                continue
            if msg.performative == Performative.ACCEPT_PROPOSAL:
                self.accepted.append(msg.sender_id)
            elif msg.performative == Performative.REJECT_PROPOSAL:
                self.rejected.append(msg.sender_id)
        super().perceive(env)

    def on_approach(self, env):
        if not self.tsp_enabled:
            return
        ahead = self.upcoming_intersections(env)
        if not ahead:
            return
        node, axis, eta = ahead[0]
        if eta <= self.propose_horizon and node not in self._proposed:
            self.send_message(node, Performative.PROPOSE,
                              {"bus_id": self.agent_id, "axis": axis, "eta_s": round(eta, 1),
                               "passengers": self.passengers},
                              ontology="transit-signal-priority", protocol="fipa-propose")
            self._proposed.add(node)

    def on_crossed(self, node: str, env):
        if node in self._proposed:
            self.send_message(node, Performative.CANCEL, {"bus_id": self.agent_id, "status": "CLEARED"},
                              ontology="transit-signal-priority", protocol="fipa-propose")
