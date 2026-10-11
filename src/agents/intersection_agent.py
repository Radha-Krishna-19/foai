"""
Intersection Agent - autonomous traffic-signal controller.

Owner: Member 2 (Intersection Agent & Signal Control)

Decision priority inside reason():
    1. Emergency preemption  (absolute; safety yellow is never skipped)
    3. Normal control: Max-Pressure (adaptive) or Fixed-Time (baseline)
Peer coordination: when a green releases a platoon, the agent INFORMs the downstream neighbour,
which adds the expected arrivals to that axis' pressure (green-wave hand-off).
"""

from typing import Any, Dict, List, Optional, Tuple
from src.agents.base_agent import BaseAgent, AgentType
from src.algorithms.max_pressure import MaxPressureController, SignalPhase, Decision, axis_of, phase_for
from src.protocols.message import MessageBroker, Performative

DEFAULT_PARAMS = dict(min_green_time=5.0, max_green_time=30.0, yellow_time=3.0,
                      starvation_weight=0.5, switch_threshold=1.0)


class IntersectionAgent(BaseAgent):
    def __init__(self, agent_id: str, broker: MessageBroker, controlled_node: str,
                 ns_incoming_roads: List[str], ns_outgoing_roads: List[str],
                 ew_incoming_roads: List[str], ew_outgoing_roads: List[str],
                 mode: str = "MAX_PRESSURE", controller_params: Optional[Dict] = None,
                 green_wave: bool = True):
        super().__init__(agent_id, AgentType.INTERSECTION, broker)
        self.controlled_node = controlled_node
        self.ns_incoming_roads = ns_incoming_roads
        self.ns_outgoing_roads = ns_outgoing_roads
        self.ew_incoming_roads = ew_incoming_roads
        self.ew_outgoing_roads = ew_outgoing_roads
        self.mode = mode
        self.green_wave = green_wave

        params = dict(DEFAULT_PARAMS)
        params.update(controller_params or {})
        self.controller = MaxPressureController(**params)

        # signal state
        self.current_phase = SignalPhase.NORTH_SOUTH_GREEN
        self.previous_green = SignalPhase.NORTH_SOUTH_GREEN
        self.current_phase_duration = 0.0
        self.time_since_ns_green = 0.0
        self.time_since_ew_green = 0.0
        self.fixed_cycle_green = 20.0

        # priority requests
        self.emergency_requests: Dict[str, Tuple[str, float]] = {}   # ev_id -> (axis, expiry time)
        self.emergency_timeout = 40.0

        # green-wave: anticipated platoon arrivals per axis -> list of (vehicles, expiry)
        self.neighbors: Dict[str, str] = {}      # neighbour id -> axis
        self.anticipated: Dict[str, List[Tuple[int, float]]] = {"NS": [], "EW": []}
        self.platoon_weight = 1.0
        self.platoon_threshold = 4

        # perception
        self.ns_in_queue = self.ns_out_queue = self.ew_in_queue = self.ew_out_queue = 0
        self.raw_ns = self.raw_ew = 0.0

        # statistics
        self.last_decision: Optional[Decision] = None
        self.phase_switches = 0
        self.longest_red = {"NS": 0.0, "EW": 0.0}
        self.preemptions_served = 0
        self.decision_log: List[Tuple[float, str, float, float, str]] = []
        self.log_decisions = False

    # ------------------------------------------------------------------
    def add_neighbor(self, neighbor_id: str, axis: str = "NS"):
        self.neighbors[neighbor_id] = axis

    @property
    def neighbor_intersections(self):
        return set(self.neighbors)

    def apply_parameters(self, params: Dict[str, float]):
        for k, v in params.items():
            if hasattr(self.controller, k):
                setattr(self.controller, k, float(v))

    def axis_of_road(self, road_id: str) -> Optional[str]:
        if road_id in self.ns_incoming_roads:
            return "NS"
        if road_id in self.ew_incoming_roads:
            return "EW"
        return None

    def is_green_for(self, road_id: str) -> bool:
        return axis_of(self.current_phase) == self.axis_of_road(road_id)

    # ---- SENSE --------------------------------------------------------
    def perceive(self, env: Dict[str, Any]):
        net = env["network"]
        now = self.sim_time

        def q(rid):
            s = net.segment_by_id(rid)
            return len(s.vehicles) if s else 0

        ns_in = [q(r) for r in self.ns_incoming_roads]
        ns_out = [q(r) for r in self.ns_outgoing_roads]
        ew_in = [q(r) for r in self.ew_incoming_roads]
        ew_out = [q(r) for r in self.ew_outgoing_roads]
        self.ns_in_queue, self.ns_out_queue = sum(ns_in), sum(ns_out)
        self.ew_in_queue, self.ew_out_queue = sum(ew_in), sum(ew_out)
        # movements: traffic from road i leaves on the opposite road (through movement)
        ns_moves = [(ns_in[0], ns_out[1]), (ns_in[1], ns_out[0])] if len(ns_in) == 2 else list(zip(ns_in, ns_out))
        ew_moves = [(ew_in[0], ew_out[1]), (ew_in[1], ew_out[0])] if len(ew_in) == 2 else list(zip(ew_in, ew_out))
        self.raw_ns = MaxPressureController.movement_pressure(ns_moves)
        self.raw_ew = MaxPressureController.movement_pressure(ew_moves)

        for msg in self.inbox:
            c = msg.content
            if msg.performative == Performative.PREEMPT_OVERRIDE:
                ev = msg.sender_id
                first = ev not in self.emergency_requests
                self.emergency_requests[ev] = (c.get("corridor_direction", "NS"), now + self.emergency_timeout)
                if first:
                    self.preemptions_served += 1
                    self.send_message(ev, Performative.AGREE,
                                      {"status": "PREEMPTION_GRANTED", "intersection": self.agent_id,
                                       "axis": c.get("corridor_direction")},
                                      ontology="emergency-preemption", in_reply_to=msg.conversation_id,
                                      conversation_id=msg.conversation_id)
            elif msg.performative == Performative.CANCEL:
                self.emergency_requests.pop(msg.sender_id, None)
            elif msg.performative == Performative.INFORM and msg.ontology == "green-wave" and self.green_wave:
                axis = c.get("axis")
                if axis in self.anticipated:
                    self.anticipated[axis].append((int(c.get("estimated_vehicles", 0)), now + c.get("eta_window", 25.0)))
            elif msg.performative == Performative.INFORM and c.get("type") == "PARAMETER_UPDATE":
                self.apply_parameters(c.get("params", {}))
        self.inbox.clear()

        # expire stale requests / predictions
        self.emergency_requests = {k: v for k, v in self.emergency_requests.items() if v[1] > now}
        for axis in self.anticipated:
            self.anticipated[axis] = [p for p in self.anticipated[axis] if p[1] > now]

    # ---- REASON -------------------------------------------------------
    def _tick_timers(self):
        self.current_phase_duration += 1.0
        a = axis_of(self.current_phase)
        if a == "NS":
            self.time_since_ns_green = 0.0
            self.time_since_ew_green += 1.0
        elif a == "EW":
            self.time_since_ew_green = 0.0
            self.time_since_ns_green += 1.0
        else:
            self.time_since_ns_green += 1.0
            self.time_since_ew_green += 1.0
        for ax, t in (("NS", self.time_since_ns_green), ("EW", self.time_since_ew_green)):
            self.longest_red[ax] = max(self.longest_red[ax], t)

    def _serve_axis(self, axis: str, can_cut_min_green: bool) -> Optional[SignalPhase]:
        """Drive the signal towards `axis` green, always via yellow."""
        target = phase_for(axis)
        if self.current_phase == target:
            return target
        if self.current_phase == SignalPhase.YELLOW_TRANSITION:
            if self.current_phase_duration >= self.controller.yellow_time:
                return target
            return SignalPhase.YELLOW_TRANSITION
        if can_cut_min_green or self.current_phase_duration >= self.controller.min_green_time:
            return SignalPhase.YELLOW_TRANSITION
        return None

    def reason(self) -> SignalPhase:
        self._tick_timers()
        ns_s = self.raw_ns + self.controller.starvation_weight * self.time_since_ns_green
        ew_s = self.raw_ew + self.controller.starvation_weight * self.time_since_ew_green

        # 1. emergency preemption
        if self.emergency_requests:
            axis = next(iter(self.emergency_requests.values()))[0]
            nxt = self._serve_axis(axis, can_cut_min_green=True)
            self.last_decision = Decision(nxt, ns_s, ew_s, f"EMERGENCY preemption for {axis}")
            return nxt

        # 3a. fixed-time baseline
        if self.mode == "FIXED_TIME":
            if self.current_phase == SignalPhase.YELLOW_TRANSITION:
                if self.current_phase_duration >= self.controller.yellow_time:
                    nxt = (SignalPhase.EAST_WEST_GREEN if self.previous_green == SignalPhase.NORTH_SOUTH_GREEN
                           else SignalPhase.NORTH_SOUTH_GREEN)
                else:
                    nxt = SignalPhase.YELLOW_TRANSITION
            elif self.current_phase_duration >= self.fixed_cycle_green:
                nxt = SignalPhase.YELLOW_TRANSITION
            else:
                nxt = self.current_phase
            self.last_decision = Decision(nxt, ns_s, ew_s, "fixed-time cycle")
            return nxt

        # 3b. Max-Pressure with green-wave anticipation
        # green-wave: expected platoon only EXTENDS the green it is heading for (never forces a switch)
        cur_axis = axis_of(self.current_phase)
        bonus_ns = self.platoon_weight * sum(v for v, _ in self.anticipated["NS"]) if cur_axis == "NS" else 0.0
        bonus_ew = self.platoon_weight * sum(v for v, _ in self.anticipated["EW"]) if cur_axis == "EW" else 0.0
        d = self.controller.decide(self.current_phase, self.current_phase_duration,
                                   self.raw_ns + bonus_ns, self.raw_ew + bonus_ew,
                                   self.ns_in_queue, self.ew_in_queue,
                                   self.time_since_ns_green, self.time_since_ew_green,
                                   previous_green=self.previous_green)
        self.last_decision = d
        return d.phase

    # ---- ACT ----------------------------------------------------------
    def act(self, env: Dict[str, Any]):
        new_phase = self.decision
        if new_phase is None:
            new_phase = self.current_phase
        if new_phase != self.current_phase:
            if axis_of(self.current_phase) is not None:
                self.previous_green = self.current_phase
            self.current_phase = new_phase
            self.current_phase_duration = 0.0
            self.phase_switches += 1
            # green-wave hand-off: a new green is about to release a platoon
            axis = axis_of(new_phase)
            if axis and self.green_wave and self.mode == "MAX_PRESSURE":
                released = self.ns_in_queue if axis == "NS" else self.ew_in_queue
                if released >= self.platoon_threshold:
                    for nb, nb_axis in self.neighbors.items():
                        if nb_axis == axis:
                            self.send_message(nb, Performative.INFORM,
                                              {"event": "PLATOON_INCOMING", "axis": axis,
                                               "estimated_vehicles": released, "eta_window": 25.0},
                                              ontology="green-wave")
        if self.log_decisions and self.last_decision is not None:
            d = self.last_decision
            self.decision_log.append((self.sim_time, self.current_phase.value, round(d.ns_score, 2),
                                      round(d.ew_score, 2), d.reason))
