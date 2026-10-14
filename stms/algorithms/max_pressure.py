"""
Max-Pressure distributed signal control (Varaiya, 2013) with an anti-starvation term.

Owner: Member 2 (Intersection Agent & Signal Control)

For every movement (l -> m) through the intersection (l = incoming road, m = outgoing road):
        w(l, m) = Q_l - Q_m                               (queue differential)
Phase pressure (only positive differentials push traffic):
        P(p) = sum_{(l,m) in p} max(0, Q_l - Q_m)
Anti-starvation (fairness) term:
        S(p) = P(p) + beta * T_red(p)                     (T_red = seconds since p was last green)
Decision rule, evaluated every second after the minimum green:
        switch to the other phase  iff  S(other) > S(current) + theta  and  other phase has waiting cars
        forced switch at max_green if the other phase has waiting cars
Every switch goes through a fixed yellow (clearance) interval.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Optional, Tuple


class SignalPhase(str, Enum):
    NORTH_SOUTH_GREEN = "NS_GREEN"
    EAST_WEST_GREEN = "EW_GREEN"
    YELLOW_TRANSITION = "YELLOW"
    EMERGENCY_PREEMPTION = "EMERGENCY_PREEMPT"


def axis_of(phase: SignalPhase) -> Optional[str]:
    if phase == SignalPhase.NORTH_SOUTH_GREEN:
        return "NS"
    if phase == SignalPhase.EAST_WEST_GREEN:
        return "EW"
    return None


def phase_for(axis: str) -> SignalPhase:
    return SignalPhase.NORTH_SOUTH_GREEN if axis == "NS" else SignalPhase.EAST_WEST_GREEN


@dataclass
class Decision:
    phase: SignalPhase
    ns_score: float
    ew_score: float
    reason: str


class MaxPressureController:
    def __init__(self, min_green_time: float = 5.0, max_green_time: float = 30.0, yellow_time: float = 3.0,
                 starvation_weight: float = 0.5, switch_threshold: float = 1.0):
        self.min_green_time = min_green_time
        self.max_green_time = max_green_time
        self.yellow_time = yellow_time
        self.starvation_weight = starvation_weight
        self.switch_threshold = switch_threshold

    @staticmethod
    def movement_pressure(movements: Iterable[Tuple[int, int]]) -> float:
        """P(p) = sum max(0, Q_in - Q_out) over the phase's movements."""
        return float(sum(max(0, q_in - q_out) for q_in, q_out in movements))

    def compute_phase_pressures(self, ns_incoming_queue: int, ns_outgoing_queue: int,
                                ew_incoming_queue: int, ew_outgoing_queue: int,
                                time_since_ns_green: float, time_since_ew_green: float) -> Tuple[float, float]:
        """Aggregate form (one movement per axis) - kept for the unit tests and the worked example."""
        raw_ns = max(0.0, float(ns_incoming_queue - ns_outgoing_queue))
        raw_ew = max(0.0, float(ew_incoming_queue - ew_outgoing_queue))
        return (raw_ns + self.starvation_weight * time_since_ns_green,
                raw_ew + self.starvation_weight * time_since_ew_green)

    def decide(self, current_phase: SignalPhase, phase_duration: float,
               raw_ns: float, raw_ew: float, ns_waiting: int, ew_waiting: int,
               time_since_ns_green: float, time_since_ew_green: float,
               previous_green: Optional[SignalPhase] = None) -> Decision:
        ns_score = raw_ns + self.starvation_weight * time_since_ns_green
        ew_score = raw_ew + self.starvation_weight * time_since_ew_green

        if current_phase == SignalPhase.YELLOW_TRANSITION:
            if phase_duration >= self.yellow_time:
                if previous_green is not None:
                    nxt = (SignalPhase.EAST_WEST_GREEN if previous_green == SignalPhase.NORTH_SOUTH_GREEN
                           else SignalPhase.NORTH_SOUTH_GREEN)
                else:
                    nxt = (SignalPhase.EAST_WEST_GREEN if time_since_ew_green >= time_since_ns_green
                           else SignalPhase.NORTH_SOUTH_GREEN)
                return Decision(nxt, ns_score, ew_score, "yellow clearance complete")
            return Decision(current_phase, ns_score, ew_score, "yellow clearance")

        if phase_duration < self.min_green_time:
            return Decision(current_phase, ns_score, ew_score, "min-green hold")

        on_ns = current_phase == SignalPhase.NORTH_SOUTH_GREEN
        cur_score, other_score = (ns_score, ew_score) if on_ns else (ew_score, ns_score)
        other_waiting = ew_waiting if on_ns else ns_waiting

        if other_waiting == 0:
            return Decision(current_phase, ns_score, ew_score, "no demand on red approach")
        if phase_duration >= self.max_green_time:
            return Decision(SignalPhase.YELLOW_TRANSITION, ns_score, ew_score, "max-green reached")
        if other_score > cur_score + self.switch_threshold:
            return Decision(SignalPhase.YELLOW_TRANSITION, ns_score, ew_score, "higher pressure on red approach")
        return Decision(current_phase, ns_score, ew_score, "current phase has higher pressure")

    def select_optimal_phase(self, current_phase: SignalPhase, current_phase_duration: float,
                             ns_incoming_queue: int, ns_outgoing_queue: int,
                             ew_incoming_queue: int, ew_outgoing_queue: int,
                             time_since_ns_green: float, time_since_ew_green: float) -> SignalPhase:
        """Convenience wrapper using aggregate queues (one movement per axis)."""
        raw_ns = max(0.0, float(ns_incoming_queue - ns_outgoing_queue))
        raw_ew = max(0.0, float(ew_incoming_queue - ew_outgoing_queue))
        return self.decide(current_phase, current_phase_duration, raw_ns, raw_ew,
                           ns_incoming_queue, ew_incoming_queue,
                           time_since_ns_green, time_since_ew_green).phase
