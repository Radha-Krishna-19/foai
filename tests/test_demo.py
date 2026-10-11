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
        done = {e["car"]: e for e in self.d["events"]}
        for n in hit:
            self.assertIn(n, done)
            self.assertFalse(uses_closed(done[n]["new"]))
            self.assertTrue(uses_closed(done[n]["old"]))
        record("DM-01", f"{len(hit)} cars had the closed road on their route -> all re-planned without it")

    def test_DM02_unaffected_cars_keep_original_route(self):
        """DM-02: cars that do not use the closed road are never re-routed"""
        free = [c["name"] for c in self.d["cars"] if not uses_closed(c["original"])]
        changed = {e["car"] for e in self.d["events"]}
        self.assertTrue(free)
        self.assertFalse(set(free) & changed)
        record("DM-02", f"{len(free)} unaffected cars kept their original route")

    def test_DM03_all_tracked_cars_arrive(self):
        """DM-03: all tracked cars reach their destination despite the accident"""
        self.assertTrue(all(t is not None for t in self.d["arrived"].values()))
        record("DM-03", "latest arrival t = %d s of %d s recorded" % (max(self.d["arrived"].values()), len(self.d["frames"])))


if __name__ == "__main__":
    unittest.main()
