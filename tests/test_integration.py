"""
INTEGRATION TESTS - agents interacting through the message broker inside a running simulation.
"""

import os
import random
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.algorithms.max_pressure import SignalPhase, axis_of
from src.protocols.message import ACLMessage, Performative
from src.scenarios import make_sim, OD_2x2
from tests._record import record


def watch_phases(sim, ticks):
    """Runs the simulation and returns every phase transition per intersection."""
    last = {k: a.current_phase for k, a in sim.intersections.items()}
    transitions = []
    for _ in range(ticks):
        sim.step()
        for k, a in sim.intersections.items():
            if a.current_phase != last[k]:
                transitions.append((sim.sim_time, k, last[k], a.current_phase))
                last[k] = a.current_phase
    return transitions


class TestAgentInteraction(unittest.TestCase):

    def test_IT01_preemption_protocol(self):
        """IT-01: ambulance PREEMPT_OVERRIDE -> intersection AGREE -> ambulance CANCEL after crossing, per intersection"""
        sim = make_sim()
        ev = sim.spawn_emergency_vehicle("S0", "N0", departure_time=5)
        sim.run_until_clear(300)
        seq = [(m.performative.value, m.sender_id, m.receiver_id) for m in sim.broker.history(ontology="emergency-preemption")]
        for node in ("INT_10", "INT_00"):
            mine = [p for p, s, r in seq if node in (s, r)]
            self.assertEqual(mine, ["PREEMPT_OVERRIDE", "AGREE", "CANCEL"])
        self.assertEqual(ev.total_wait_time, 0)
        record("IT-01", "INT_10 and INT_00 each: PREEMPT_OVERRIDE, AGREE, CANCEL; ambulance wait 0 s")

    def test_IT02_yellow_safety_invariant(self):
        """IT-02: SAFETY - no signal ever goes directly from one green to the other (always via YELLOW), even with preemption and bus priority"""
        sim = make_sim()
        sim.spawn_poisson_demand(OD_2x2, 0.45, 250, random.Random(7))
        for t in (30, 90, 150):
            sim.spawn_emergency_vehicle("S0", "N0", departure_time=t)
            sim.spawn_emergency_vehicle("W1", "E1", departure_time=t + 10)
        sim.spawn_bus("W0", "E0", departure_time=40)
        sim.spawn_bus("N1", "S1", departure_time=60)
        trans = watch_phases(sim, 400)
        bad = [t for t in trans if axis_of(t[2]) and axis_of(t[3])]
        self.assertEqual(bad, [])
        short_yellow = []
        starts = {}
        for t, k, a, b in trans:
            if b == SignalPhase.YELLOW_TRANSITION:
                starts[k] = t
            elif a == SignalPhase.YELLOW_TRANSITION:
                if t - starts.get(k, -99) < 3:
                    short_yellow.append((k, t))
        self.assertEqual(short_yellow, [])
        record("IT-02", f"{len(trans)} phase changes checked: 0 green->green, 0 yellows shorter than 3 s")

    def test_IT03_tsp_accept_and_reject(self):
        """IT-03: bus priority is ACCEPTED on a light cross street and REJECTED on a heavy one"""
        sim = make_sim()
        agent = sim.intersections["INT_00"]
        sim.broker.register_agent("BUS_X")
        env = sim.env_state()
        for light in (True, False):
            seg = sim.network.segment_by_id("N0->INT_00")
            seg.vehicles = [] if light else [f"d{i}" for i in range(10)]
            sim.broker.send(ACLMessage("BUS_X", "INT_00", Performative.PROPOSE, {"axis": "EW"},
                                       ontology="transit-signal-priority"))
            agent.step(env)
        replies = [m.performative for m in sim.broker.receive("BUS_X")]
        self.assertEqual(replies, [Performative.ACCEPT_PROPOSAL, Performative.REJECT_PROPOSAL])
        record("IT-03", "cross queue 0 -> ACCEPT_PROPOSAL; cross queue 10 -> REJECT_PROPOSAL")

    def test_IT04_tsp_rejected_during_emergency(self):
        """IT-04: an ambulance preemption outranks bus priority (bus proposal rejected)"""
        sim = make_sim()
        agent = sim.intersections["INT_00"]
        for a in ("AMB_X", "BUS_X"):
            sim.broker.register_agent(a)
        sim.broker.send(ACLMessage("AMB_X", "INT_00", Performative.PREEMPT_OVERRIDE, {"corridor_direction": "NS"},
                                   ontology="emergency-preemption"))
        sim.broker.send(ACLMessage("BUS_X", "INT_00", Performative.PROPOSE, {"axis": "EW"},
                                   ontology="transit-signal-priority"))
        agent.step(sim.env_state())
        rep = sim.broker.receive("BUS_X")[0]
        self.assertEqual(rep.performative, Performative.REJECT_PROPOSAL)
        self.assertIn("emergency", rep.content["reason"])
        record("IT-04", f"bus got REJECT_PROPOSAL: '{rep.content['reason']}'")

    def test_IT05_incident_broadcast_reroute(self):
        """IT-05: TMC INCIDENT_ALERT makes every affected vehicle drop the closed road from its plan"""
        sim = make_sim()
        vs = [sim.spawn_vehicle("W0", "E0", departure_time=i * 2) for i in range(6)]
        sim.run(3)
        sim.coordinator.report_incident("INT_00", "INT_01")
        sim.run(2)
        uses = [v for v in vs if any(a == "INT_00" and b == "INT_01" for a, b in zip(v.planned_path, v.planned_path[1:]))]
        self.assertEqual(uses, [])
        sim.run_until_clear(400)
        self.assertTrue(all(v.has_arrived for v in vs))
        record("IT-05", "6 / 6 vehicles re-planned away from INT_00->INT_01 and arrived")

    def test_IT06_parameter_update(self):
        """IT-06: coordinator INFORM PARAMETER_UPDATE changes every intersection's controller"""
        sim = make_sim()
        sim.coordinator.intersection_ids = list(sim.intersections)
        for iid in sim.intersections:
            sim.coordinator.send_message(iid, Performative.INFORM,
                                         {"type": "PARAMETER_UPDATE", "params": {"min_green_time": 8, "max_green_time": 40}})
        sim.step()
        vals = {(a.controller.min_green_time, a.controller.max_green_time) for a in sim.intersections.values()}
        self.assertEqual(vals, {(8.0, 40.0)})
        record("IT-06", "all 4 intersections now min_green 8 s, max_green 40 s")

    def test_IT07_green_wave_handoff(self):
        """IT-07: releasing a platoon sends INFORM PLATOON_INCOMING to the downstream neighbour, which stores it"""
        sim = make_sim(1, 3)
        for i in range(10):
            sim.spawn_vehicle("W0", "E0", departure_time=0)
        sim.run(80)
        msgs = sim.broker.history(ontology="green-wave")
        self.assertGreater(len(msgs), 0)
        self.assertTrue(all(m.content["axis"] in ("NS", "EW") for m in msgs))
        record("IT-07", f"{len(msgs)} PLATOON_INCOMING messages exchanged on a 1x3 corridor")


class TestSimulationInvariants(unittest.TestCase):

    def test_IT08_conservation_and_capacity(self):
        """IT-08: every vehicle is on at most one road, no road exceeds capacity, and all vehicles arrive"""
        sim = make_sim(3, 3)
        from src.scenarios import boundary_od_pairs
        sim.spawn_poisson_demand(boundary_od_pairs(3, 3), 0.6, 300, random.Random(2))
        worst = 0
        while not sim.all_done() and sim.sim_time < 3000:
            sim.step()
            seen = {}
            for seg in sim.network.segments.values():
                self.assertLessEqual(len(seg.vehicles), seg.capacity)
                worst = max(worst, len(seg.vehicles) / seg.capacity)
                for v in seg.vehicles:
                    self.assertNotIn(v, seen)
                    seen[v] = seg.segment_id
        self.assertTrue(sim.all_done())
        record("IT-08", f"{len(sim.vehicles)} vehicles, all arrived at t={sim.sim_time:.0f} s; peak road occupancy {worst:.0%}")

    def test_IT09_determinism(self):
        """IT-09: the same seed gives exactly the same result (reproducible experiments)"""
        out = []
        for _ in range(2):
            sim = make_sim()
            sim.spawn_poisson_demand(OD_2x2, 0.35, 200, random.Random(11))
            sim.run_until_clear(2000)
            out.append(sim.summary())
        self.assertEqual(out[0], out[1])
        record("IT-09", f"two runs identical: avg wait {out[0]['avg_wait_s']} s, {out[0]['messages']} messages")


if __name__ == "__main__":
    unittest.main(verbosity=2)
