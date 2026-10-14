"""
UNIT TESTS - each algorithm / component in isolation.
Run:  python -m unittest discover -s tests -v      (or: python tests/run_tests.py for the report)
"""

import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from stms.environment.road_network import RoadNetwork, RoadSegment
from stms.environment.traffic_grid import build_grid_network
from stms.algorithms.search_astar import AStarRouter
from stms.algorithms.max_pressure import MaxPressureController, SignalPhase
from stms.algorithms.hill_climbing import ParameterSpace, hill_climb
from stms.protocols.message import ACLMessage, MessageBroker, Performative
from stms.scenarios import congested_grid, make_sim
from tests._record import record


def small_net():
    net = RoadNetwork()
    for n, x, y in (("A", 0, 0), ("B", 100, 0), ("C", 200, 0), ("D", 100, 100)):
        net.add_intersection(n, x, y)
    for u, v in (("A", "B"), ("B", "C"), ("A", "D"), ("D", "C")):
        net.add_road(u, v, speed_limit_mps=10.0, capacity=10)
    return net


class TestSearch(unittest.TestCase):
    """Member 3 - route search"""

    @classmethod
    def setUpClass(cls):
        cls.sim = congested_grid(6, seed=5)
        cls.net = cls.sim.network
        cls.astar = AStarRouter(cls.net, algorithm="astar")
        rng = random.Random(1)
        ends = [n for n in cls.net.node_positions if not n.startswith("INT")]
        cls.pairs = [tuple(rng.sample(ends, 2)) for _ in range(60)]

    def test_UT01_heuristic_admissible(self):
        """UT-01: h(n) <= true optimal cost h*(n) for every node and goal tested"""
        ucs = AStarRouter(self.net, algorithm="ucs")
        worst = 0.0
        for goal in ["E3", "S0", "N5", "W2"]:
            for n in self.net.node_positions:
                true = ucs.search(n, goal).cost
                if true == math.inf:
                    continue
                h = self.astar.heuristic(n, goal)
                self.assertLessEqual(h, true + 1e-9)
                worst = max(worst, h / true if true else 0)
        record("UT-01", f"max h/h* ratio = {worst:.3f} (<= 1 required)")

    def test_UT02_heuristic_consistent(self):
        """UT-02: h(u) <= c(u,v) + h(v) for every road and goal (consistency / monotonicity)"""
        checked = 0
        for goal in ["E3", "S0", "N5"]:
            for seg in self.net.segments.values():
                c = seg.current_traversal_time()
                if c == math.inf:
                    continue
                self.assertLessEqual(self.astar.heuristic(seg.source, goal),
                                     c + self.astar.heuristic(seg.target, goal) + 1e-9)
                checked += 1
        record("UT-02", f"{checked} (edge, goal) pairs satisfy the triangle inequality")

    def test_UT03_astar_optimal(self):
        """UT-03: A* returns the same optimal cost as UCS/Dijkstra on 60 random trips"""
        ucs = AStarRouter(self.net, algorithm="ucs")
        for o, d in self.pairs:
            self.assertAlmostEqual(self.astar.search(o, d).cost, ucs.search(o, d).cost, places=6)
        record("UT-03", "60 / 60 trips: cost(A*) == cost(UCS)")

    def test_UT04_astar_expands_fewer(self):
        """UT-04: A* never expands more nodes than UCS (same tie-breaking)"""
        ucs = AStarRouter(self.net, algorithm="ucs")
        a = sum(self.astar.search(o, d).nodes_expanded for o, d in self.pairs)
        u = sum(ucs.search(o, d).nodes_expanded for o, d in self.pairs)
        self.assertLess(a, u)
        record("UT-04", f"total expansions A* = {a}, UCS = {u}")

    def test_UT05_reroute_on_blockage(self):
        """UT-05: when A->B is closed, A* routes A->D->C"""
        net = small_net()
        r = AStarRouter(net)
        self.assertEqual(r.search("A", "C").path, ["A", "B", "C"])
        net.set_incident("A", "B", True)
        self.assertEqual(r.search("A", "C").path, ["A", "D", "C"])
        record("UT-05", "path before: A-B-C, after closure: A-D-C")

    def test_UT06_bfs_fewest_edges(self):
        """UT-06: BFS returns a fewest-roads path"""
        r = AStarRouter(self.net, algorithm="bfs")
        res = r.search("W0", "E0")
        self.assertEqual(len(res.path) - 1, 7)       # W0 + 6 intersections + E0
        record("UT-06", f"W0->E0 on 6x6 grid: {len(res.path) - 1} roads (minimum = 7)")

    def test_UT07_unreachable(self):
        """UT-07: search reports failure (empty path, infinite cost) when the goal is cut off"""
        net = small_net()
        net.set_incident("B", "C", True)
        net.set_incident("D", "C", True)
        for algo in ("astar", "ucs", "bfs", "greedy"):
            res = AStarRouter(net, algorithm=algo).search("A", "C")
            self.assertEqual(res.path, [])
        record("UT-07", "all 4 algorithms return [] for an unreachable goal")

    def test_UT08_greedy_valid_path(self):
        """UT-08: Greedy best-first returns a valid (connected) path, possibly sub-optimal"""
        r = AStarRouter(self.net, algorithm="greedy")
        for o, d in self.pairs[:20]:
            p = r.search(o, d).path
            for a, b in zip(p, p[1:]):
                self.assertIsNotNone(self.net.get_segment(a, b))
        record("UT-08", "20 / 20 greedy paths use only existing roads")


class TestRoadModel(unittest.TestCase):
    """Member 1 - environment"""

    def test_UT09_bpr(self):
        """UT-09: BPR travel time grows with load, equals free-flow when empty, infinite when closed"""
        s = RoadSegment("x", "a", "b", 200.0, 10.0, capacity=10)
        t0 = s.current_traversal_time()
        s.vehicles = [str(i) for i in range(10)]
        t1 = s.current_traversal_time()
        self.assertAlmostEqual(t0, 20.0)
        self.assertAlmostEqual(t1, 20.0 * 1.15)
        s.is_blocked = True
        self.assertEqual(s.current_traversal_time(), math.inf)
        record("UT-09", f"empty {t0:.1f}s, full {t1:.1f}s, closed inf")

    def test_UT10_capacity(self):
        """UT-10: a road refuses vehicles when full or closed"""
        s = RoadSegment("x", "a", "b", 100.0, capacity=2)
        self.assertTrue(s.add_vehicle("v1") and s.add_vehicle("v2"))
        self.assertFalse(s.add_vehicle("v3"))
        s.remove_vehicle("v1")
        s.is_blocked = True
        self.assertFalse(s.add_vehicle("v4"))
        record("UT-10", "3rd vehicle refused at capacity 2; refused when closed")

    def test_UT11_saturation_headway(self):
        """UT-11: stop line discharges at most one vehicle every 2 s (1800 veh/h/lane)"""
        s = RoadSegment("x", "a", "b", 100.0)
        s.mark_discharge(10.0)
        self.assertFalse(s.can_discharge(11.0))
        self.assertTrue(s.can_discharge(12.0))
        record("UT-11", "discharge blocked at +1 s, allowed at +2 s")

    def test_UT12_grid_generator(self):
        """UT-12: an R x C grid has R*C intersection agents and the right number of roads"""
        for r, c in ((2, 2), (3, 4), (6, 6)):
            net, agents = build_grid_network(MessageBroker(), r, c)
            self.assertEqual(len(agents), r * c)
            internal = 2 * (r * (c - 1) + c * (r - 1))
            feeders = 2 * (2 * r + 2 * c)
            self.assertEqual(len(net.segments), internal + feeders)
        record("UT-12", "2x2, 3x4, 6x6 grids: agent and road counts correct")


class TestMaxPressure(unittest.TestCase):
    """Member 2 - signal control"""

    def setUp(self):
        self.c = MaxPressureController(min_green_time=5, max_green_time=30, yellow_time=3,
                                       starvation_weight=0.5, switch_threshold=1.0)

    def test_UT13_switch_to_higher_pressure(self):
        """UT-13: after min-green, switch (via YELLOW) when the red approach has higher pressure"""
        p = self.c.select_optimal_phase(SignalPhase.EAST_WEST_GREEN, 10, 20, 0, 2, 0, 20, 0)
        self.assertEqual(p, SignalPhase.YELLOW_TRANSITION)
        record("UT-13", "EW green, NS pressure 20+10 vs EW 2 -> YELLOW")

    def test_UT14_min_green_hold(self):
        """UT-14: never switch before the minimum green time"""
        p = self.c.select_optimal_phase(SignalPhase.EAST_WEST_GREEN, 2, 20, 0, 0, 0, 20, 0)
        self.assertEqual(p, SignalPhase.EAST_WEST_GREEN)
        record("UT-14", "at 2 s (< 5 s min green) phase held")

    def test_UT15_max_green(self):
        """UT-15: forced switch at max-green if the other approach has waiting cars"""
        p = self.c.select_optimal_phase(SignalPhase.NORTH_SOUTH_GREEN, 30, 50, 0, 1, 0, 0, 0)
        self.assertEqual(p, SignalPhase.YELLOW_TRANSITION)
        record("UT-15", "NS green 30 s with 1 EW car waiting -> YELLOW")

    def test_UT16_no_switch_to_empty(self):
        """UT-16: no switch towards an approach with no waiting vehicles (no wasted green)"""
        p = self.c.select_optimal_phase(SignalPhase.NORTH_SOUTH_GREEN, 60, 3, 0, 0, 0, 0, 60)
        self.assertEqual(p, SignalPhase.NORTH_SOUTH_GREEN)
        record("UT-16", "EW empty -> NS kept green even past max-green")

    def test_UT17_yellow_then_opposite(self):
        """UT-17: yellow lasts yellow_time and is followed by the OPPOSITE green"""
        d = self.c.decide(SignalPhase.YELLOW_TRANSITION, 2, 0, 5, 0, 5, 3, 0,
                          previous_green=SignalPhase.NORTH_SOUTH_GREEN)
        self.assertEqual(d.phase, SignalPhase.YELLOW_TRANSITION)
        d = self.c.decide(SignalPhase.YELLOW_TRANSITION, 3, 0, 5, 0, 5, 3, 0,
                          previous_green=SignalPhase.NORTH_SOUTH_GREEN)
        self.assertEqual(d.phase, SignalPhase.EAST_WEST_GREEN)
        record("UT-17", "yellow held at 2 s, NS->YELLOW->EW at 3 s")

    def test_UT18_pressure_formula(self):
        """UT-18: P = sum max(0, Q_in - Q_out) over movements (worked example)"""
        self.assertEqual(MaxPressureController.movement_pressure([(8, 2), (3, 5)]), 6.0)
        ns, ew = self.c.compute_phase_pressures(8, 2, 3, 1, 0, 10)
        self.assertEqual((ns, ew), (6.0, 2.0 + 0.5 * 10))
        record("UT-18", "movements (8,2),(3,5) -> 6; EW with 10 s red -> 2 + 5 = 7")


class TestProtocolAndTuning(unittest.TestCase):
    """Member 5 - protocols and hill climbing"""

    def test_UT19_point_to_point_and_reply(self):
        """UT-19: point-to-point delivery and reply threading (same conversation id)"""
        b = MessageBroker()
        b.register_agent("A")
        b.register_agent("B")
        m = ACLMessage("A", "B", Performative.REQUEST, {"x": 1})
        b.send(m)
        got = b.receive("B")
        self.assertEqual(len(got), 1)
        rep = got[0].create_reply(Performative.AGREE, {"ok": True})
        b.send(rep)
        back = b.receive("A")[0]
        self.assertEqual(back.conversation_id, m.conversation_id)
        self.assertEqual(back.performative, Performative.AGREE)
        record("UT-19", f"REQUEST/AGREE share conversation {m.conversation_id}")

    def test_UT20_broadcast(self):
        """UT-20: broadcast reaches everyone except the sender; unknown receiver is reported"""
        b = MessageBroker()
        for a in "ABC":
            b.register_agent(a)
        b.send(ACLMessage("A", "BROADCAST", Performative.BROADCAST, {}))
        self.assertEqual((len(b.receive("A")), len(b.receive("B")), len(b.receive("C"))), (0, 1, 1))
        self.assertFalse(b.send(ACLMessage("A", "ZZZ", Performative.INFORM, {})))
        record("UT-20", "B and C received, A (sender) did not; unknown receiver -> False")

    def test_UT21_hill_climbing_toy(self):
        """UT-21: hill climbing reaches the global minimum of a convex test function"""
        space = ParameterSpace({"a": list(range(10)), "b": list(range(10))})
        res = hill_climb(space, lambda p: (p["a"] - 7) ** 2 + (p["b"] - 2) ** 2, (0, 9), restarts=0, max_iters=30)
        self.assertEqual(res.best_params, {"a": 7, "b": 2})
        self.assertEqual(len(space.neighbours((0, 0))), 2)
        self.assertEqual(len(space.neighbours((5, 5))), 4)
        record("UT-21", f"found (7,2) from (0,9) in {len(res.trace) - 1} iterations")


class TestVehicle(unittest.TestCase):
    """Member 3 - vehicle agent"""

    def test_UT22_departure_time(self):
        """UT-22: a vehicle is not on the road before its departure time"""
        sim = make_sim()
        v = sim.spawn_vehicle("W0", "E0", departure_time=20)
        sim.run(19)
        self.assertFalse(v.departed)
        self.assertIsNone(v.current_segment)
        sim.run(2)
        self.assertTrue(v.departed)
        record("UT-22", "not departed at t=19, departed at t=21")

    def test_UT23_beliefs(self):
        """UT-23: learning ROAD_BLOCKED makes the road cost infinite for that vehicle; ROAD_CLEARED restores it"""
        sim = make_sim()
        v = sim.spawn_vehicle("W0", "E0")
        seg = sim.network.get_segment("INT_00", "INT_01")
        v.learn({"status": "ROAD_BLOCKED", "blocked_segment": ["INT_00", "INT_01"]})
        self.assertEqual(v.edge_weight(seg), math.inf)
        v.learn({"status": "ROAD_CLEARED", "cleared_segment": ["INT_00", "INT_01"]})
        self.assertLess(v.edge_weight(seg), 20)
        record("UT-23", "cost inf after ROAD_BLOCKED, finite after ROAD_CLEARED")


if __name__ == "__main__":
    unittest.main(verbosity=2)
