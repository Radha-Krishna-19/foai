"""
Coordinator Agent - Traffic Management Centre (TMC) supervisor.

Owner: Member 5 (Coordinator Agent & Optimisation)

Responsibilities
    1. Network telemetry  : congestion index = mean road density x 100, bottleneck list (density > 0.8)
    2. Incident management: BROADCAST INCIDENT_ALERT (road blocked / cleared) so vehicles re-plan with A*
    3. Congestion advisory: INFORM CONGESTION_WARNING (rate-limited) so vehicles consider detours
    4. Policy tuning      : runs Hill Climbing over the Max-Pressure parameters and pushes the best
                            parameters to every intersection with INFORM PARAMETER_UPDATE
The coordinator never switches a signal itself - local control stays decentralised.
"""

from typing import Any, Callable, Dict, List, Optional, Tuple
from stms.agents.base_agent import BaseAgent, AgentType
from stms.algorithms.hill_climbing import ParameterSpace, SIGNAL_PARAMETER_GRID, hill_climb, HillClimbResult
from stms.environment.road_network import RoadNetwork
from stms.protocols.message import MessageBroker, Performative


class CentralCoordinatorAgent(BaseAgent):
    def __init__(self, agent_id: str, broker: MessageBroker, network: RoadNetwork,
                 congestion_threshold: float = 45.0, warning_cooldown: float = 20.0):
        super().__init__(agent_id, AgentType.COORDINATOR, broker)
        self.network = network
        self.active_incidents: List[Tuple[str, str]] = []
        self.slowdowns: Dict[Tuple[str, str], float] = {}
        self.bottleneck_segments: List[str] = []
        self.network_congestion_index = 0.0
        self.congestion_threshold = congestion_threshold
        self.warning_cooldown = warning_cooldown
        self._last_warning = -1e9
        self.total_dispatched_advisories = 0
        self.congestion_warnings = 0
        self.intersection_ids: List[str] = []
        self.tuning_result: Optional[HillClimbResult] = None

    # ---- incident management -----------------------------------------
    def report_incident(self, source: str, target: str):
        inc = (source, target)
        if inc in self.active_incidents:
            return
        self.active_incidents.append(inc)
        self.network.set_incident(source, target, True)
        self.broadcast(Performative.BROADCAST,
                       {"type": "INCIDENT_ALERT", "status": "ROAD_BLOCKED", "blocked_segment": list(inc),
                        "recommended_action": "REPLAN_ROUTE_ASTAR"},
                       ontology="traffic-incident-management")
        self.total_dispatched_advisories += 1

    def report_slowdown(self, source: str, target: str, speed_mps: float = 2.0):
        """Partial incident: road stays open but traffic crawls (e.g. one lane closed by a crash)."""
        inc = (source, target)
        if inc in self.active_incidents:
            return
        self.active_incidents.append(inc)
        self.slowdowns[inc] = speed_mps
        self.network.set_slowdown(source, target, speed_mps)
        self.broadcast(Performative.BROADCAST,
                       {"type": "INCIDENT_ALERT", "status": "ROAD_SLOWED", "segment": list(inc),
                        "speed_mps": speed_mps, "recommended_action": "REPLAN_ROUTE_ASTAR"},
                       ontology="traffic-incident-management")
        self.total_dispatched_advisories += 1

    def current_advisories(self) -> List[Dict[str, Any]]:
        """What a newly connected vehicle is told about incidents that are still active."""
        out = []
        for inc in self.active_incidents:
            if inc in self.slowdowns:
                out.append({"status": "ROAD_SLOWED", "segment": list(inc), "speed_mps": self.slowdowns[inc]})
            else:
                out.append({"status": "ROAD_BLOCKED", "blocked_segment": list(inc)})
        return out

    def clear_incident(self, source: str, target: str):
        inc = (source, target)
        if inc not in self.active_incidents:
            return
        self.active_incidents.remove(inc)
        self.network.set_incident(source, target, False)
        self.network.set_slowdown(source, target, None)
        self.slowdowns.pop(inc, None)
        self.broadcast(Performative.BROADCAST,
                       {"type": "INCIDENT_ALERT", "status": "ROAD_CLEARED", "cleared_segment": list(inc),
                        "recommended_action": "RESTORE_NORMAL_FLOW"},
                       ontology="traffic-incident-management")
        self.total_dispatched_advisories += 1

    # ---- policy tuning (hill climbing) --------------------------------
    def tune_signal_policy(self, cost_fn: Callable[[Dict[str, float]], float], start_params: Dict[str, float],
                           restarts: int = 2, max_iters: int = 12, seed: int = 7) -> HillClimbResult:
        space = ParameterSpace(SIGNAL_PARAMETER_GRID)
        res = hill_climb(space, cost_fn, space.index_of(start_params), restarts=restarts,
                         max_iters=max_iters, seed=seed)
        self.tuning_result = res
        for iid in self.intersection_ids:
            self.send_message(iid, Performative.INFORM, {"type": "PARAMETER_UPDATE", "params": res.best_params},
                              ontology="policy-tuning")
        return res

    # ---- agent cycle ----------------------------------------------------
    def perceive(self, env: Dict[str, Any]):
        self.intersection_ids = list(env.get("intersections", {}).keys())
        self.bottleneck_segments = [s.segment_id for s in self.network.segments.values() if s.density > 0.8]
        busy = [s.density for s in self.network.segments.values() if s.vehicle_count > 0]
        # index over roads that carry traffic, so empty feeder roads do not hide congestion
        self.network_congestion_index = 100.0 * (sum(busy) / len(busy) if busy else 0.0)
        self.inbox.clear()

    def reason(self) -> Optional[str]:
        congested = self.network_congestion_index > self.congestion_threshold or bool(self.bottleneck_segments)
        if congested and self.sim_time - self._last_warning >= self.warning_cooldown:
            return "WARN"
        return None

    def act(self, env: Dict[str, Any]):
        if self.decision == "WARN":
            self._last_warning = self.sim_time
            self.congestion_warnings += 1
            self.broadcast(Performative.INFORM,
                           {"type": "CONGESTION_WARNING", "congestion_index": round(self.network_congestion_index, 1),
                            "hotspots": self.bottleneck_segments[:10]},
                           ontology="macro-flow-advisory")
