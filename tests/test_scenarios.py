"""
SYSTEM / ACCEPTANCE TESTS - each evaluation scenario must meet its acceptance criterion.
(Fewer seeds than the full report run so the suite stays under ~1 minute.)
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src import scenarios as S
from tests._record import record


class TestScenarios(unittest.TestCase):

    def test_ST01_maxpressure_beats_fixed(self):
        """ST-01 (S1): Max-Pressure reduces average wait by at least 25% vs Fixed-Time, all vehicles arrive"""
        r = S.scenario_1_fixed_vs_maxpressure(seeds=range(1, 6))
        f, m = r["rows"]["Fixed-Time (20s green)"], r["rows"]["Max-Pressure, no green-wave"]
        self.assertGreaterEqual(r["wait_reduction_pct"], 25)
        self.assertEqual(f["arrived"], f["spawned"])
        self.assertEqual(m["arrived"], m["spawned"])
        record("ST-01", f"{f['avg_wait_s']} s -> {m['avg_wait_s']} s ({r['wait_reduction_pct']}% less)")

    def test_ST03_no_starvation(self):
        """ST-03 (S2): with anti-starvation no minority vehicle is left unserved; without it some are"""
        r = S.scenario_2_rush_hour(seeds=range(1, 4))["rows"]
        self.assertEqual(r["Max-Pressure WITH anti-starvation"]["ew_vehicles_starved"], 0)
        self.assertGreater(r["Max-Pressure WITHOUT anti-starvation"]["ew_vehicles_starved"], 0)
        record("ST-03", f"starved E-W cars: with = 0, without = {r['Max-Pressure WITHOUT anti-starvation']['ew_vehicles_starved']}")

    def test_ST04_emergency(self):
        """ST-04 (S3): preemption gives the ambulance zero signal delay, >= 20% faster, < 1 s cost to others"""
        r = S.scenario_3_emergency(seeds=range(1, 6))
        self.assertEqual(r["with_preemption"]["ev_wait_s"], 0)
        self.assertGreaterEqual(r["time_saved_pct"], 20)
        self.assertLess(r["background_cost_s"], 1.0)
        record("ST-04", f"{r['without_preemption']['ev_travel_s']} s -> {r['with_preemption']['ev_travel_s']} s "
                        f"({r['time_saved_pct']}% faster), others {r['background_cost_s']:+} s")

    def test_ST05_incident(self):
        """ST-05 (S4): closure - static routing strands vehicles, re-routing delivers 100%; slowdown - broadcast >= 20% faster than local"""
        r = S.scenario_4_incident(seeds=range(1, 4))["rows"]
        self.assertEqual(r["closure"]["static"]["arrived_pct"], 0)
        self.assertEqual(r["closure"]["broadcast"]["arrived_pct"], 100)
        loc, bc = r["slowdown"]["local"]["avg_travel_s"], r["slowdown"]["broadcast"]["avg_travel_s"]
        self.assertLess(bc, 0.8 * loc)
        record("ST-05", f"closure: static 0%, broadcast 100%; slowdown: local {loc} s vs broadcast {bc} s")

    def test_ST07_search(self):
        """ST-07 (S7): A* is 100% optimal and expands fewer nodes than UCS; Greedy is not always optimal"""
        r = S.scenario_7_search_comparison(pairs=100)["rows"]
        a, u, g = r["A* (Euclidean h)"], r["UCS (Dijkstra)"], r["Greedy Best-First"]
        self.assertEqual(a["optimal_pct"], 100.0)
        self.assertLess(a["avg_nodes_expanded"], u["avg_nodes_expanded"])
        self.assertLess(g["optimal_pct"], 100.0)
        record("ST-07", f"A* {a['avg_nodes_expanded']} vs UCS {u['avg_nodes_expanded']} nodes; greedy optimal {g['optimal_pct']}%")

    def test_ST08_hill_climbing(self):
        """ST-08 (S6): hill-climbing-tuned parameters beat the defaults on unseen validation traffic"""
        r = S.scenario_6_hill_climbing()
        self.assertLess(r["train_tuned_wait_s"], r["train_default_wait_s"])
        self.assertLess(r["val_tuned_wait_s"], r["val_default_wait_s"])
        self.assertEqual(r["applied_on_INT_00"], {k: float(v) for k, v in r["tuned_params"].items()})
        record("ST-08", f"validation wait {r['val_default_wait_s']} s -> {r['val_tuned_wait_s']} s; params pushed to agents")

    def test_ST09_scalability(self):
        """ST-09 (S8): 2x2-4x4 grids all clear, Max-Pressure beats Fixed-Time at every size"""
        r = S.scenario_8_scalability(sizes=(2, 3, 4))["rows"]
        for k, v in r.items():
            self.assertEqual(v["arrived_pct"], 100.0)
            self.assertLess(v["mp_avg_wait_s"], v["fixed_avg_wait_s"])
        record("ST-09", "; ".join(f"{k}: {v['wait_reduction_pct']}% less wait, {v['ms_per_tick']} ms/tick" for k, v in r.items()))


    def test_ST06_conventional_vs_ours(self):
        """ST-06 (S5): vs a conventional city, our system moves the ambulance faster and cuts average wait"""
        r = S.scenario_5_conventional_vs_ours(seeds=range(1, 4))
        c, o = r["rows"]["Conventional"], r["rows"]["Ours (STMS)"]
        self.assertLess(o["ambulance_trip_s"], c["ambulance_trip_s"])
        self.assertLess(o["avg_wait_s"], c["avg_wait_s"])
        record("ST-06", f"ambulance {c['ambulance_trip_s']} -> {o['ambulance_trip_s']} s, avg wait {c['avg_wait_s']} -> {o['avg_wait_s']} s")


if __name__ == "__main__":
    unittest.main(verbosity=2)
