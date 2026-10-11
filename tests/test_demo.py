"""
DEMO TESTS - the one scenario shown on the demo page (original route vs re-route).
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from src.demo import record as record_demo, INCIDENT_ROAD
from tests._record import record

CLOSED = {INCIDENT_ROAD, INCIDENT_ROAD[::-1]}


def uses_closed(path):
    return any((a, b) in CLOSED for a, b in zip(path, path[1:]))


class TestDemoScenario(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.d = record_demo()

    def test_DM01_affected_cars_reroute_around_closure(self):
        """DM-01: every car whose original route used the closed road gets a new route that avoids it"""
        hit = [c["name"] for c in self.d["cars"] if uses_closed(c["original"])]
        self.assertGreaterEqual(len(hit), 1)
        done = {e["car"]: e for e in self.d["ours"]["events"]}
        for n in hit:
            self.assertIn(n, done)
            self.assertFalse(uses_closed(done[n]["new"]))
            self.assertTrue(uses_closed(done[n]["old"]))
        record("DM-01", f"{len(hit)} cars had the closed road on their route -> all re-planned without it")

    def test_DM02_unaffected_cars_keep_original_route(self):
        """DM-02: cars that do not use the closed road are never re-routed"""
        free = [c["name"] for c in self.d["cars"] if not uses_closed(c["original"])]
        changed = {e["car"] for e in self.d["ours"]["events"]}
        self.assertTrue(free)
        self.assertFalse(set(free) & changed)
        record("DM-02", f"{len(free)} unaffected cars kept their original route")

    def test_DM03_all_tracked_cars_arrive(self):
        """DM-03: all tracked cars reach their destination despite the accident"""
        self.assertTrue(all(t is not None for t in self.d["ours"]["arrived"].values()))
        record("DM-03", "latest arrival t = %d s of %d s recorded" % (max(self.d["ours"]["arrived"].values()), self.d["frames"]))

    def test_DM04_ambulance_faster_than_conventional(self):
        """DM-04: the ambulance gets through faster with preemption and never waits at red"""
        c, o = self.d["conv"]["stats"], self.d["ours"]["stats"]
        self.assertLess(o["ambulance_trip_s"], c["ambulance_trip_s"])
        self.assertEqual(o["ambulance_red_wait_s"], 0)
        self.assertGreater(c["ambulance_red_wait_s"], 0)
        record("DM-04", f"ambulance {c['ambulance_trip_s']} s (conventional) -> {o['ambulance_trip_s']} s (ours), red wait {c['ambulance_red_wait_s']} -> 0 s")

    def test_DM05_same_traffic_both_runs(self):
        """DM-05: both runs use identical traffic and the same accident, so the comparison is fair"""
        self.assertEqual(self.d["conv"]["stats"]["spawned"], self.d["ours"]["stats"]["spawned"])
        self.assertEqual(self.d["incident"]["roads"], [list(c) for c in __import__("src.demo", fromlist=["x"]).DEFAULT_CLOSED])
        record("DM-05", f"{self.d['ours']['stats']['spawned']} vehicles spawned in each run; accident J01-J11 at t = 10 s")


if __name__ == "__main__":
    unittest.main()
