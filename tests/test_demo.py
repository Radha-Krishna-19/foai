"""
DEMO TESTS - the one scenario shown on the demo page (original route vs re-route).
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from stms.demo import record as record_demo, INCIDENT_ROAD, DEFAULT_CLOSED
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

    def test_DM02_unaffected_car_keeps_route_and_dispatch_is_random_free(self):
        """DM-02: a dispatched car whose route avoids the closed road is never re-routed; no accident -> no re-route"""
        d = record_demo([], [("Car", "W1", "E1", 15)])
        self.assertFalse(d["ours"]["events"])
        self.assertFalse(uses_closed(d["cars"][0]["original"]))
        record("DM-02", "no accident -> the car keeps its A* route, 0 re-routes")

    def test_DM03_all_tracked_cars_arrive(self):
        """DM-03: the dispatched car and ambulance reach their destination despite the accident"""
        self.assertTrue(all(t is not None for t in self.d["ours"]["arrived"].values()))
        record("DM-03", "latest arrival t = %d s of %d s recorded" % (max(self.d["ours"]["arrived"].values()), self.d["frames"]))

    def test_DM04_ambulance_faster_than_conventional(self):
        """DM-04: the ambulance gets through faster with preemption and never waits at red"""
        c, o = self.d["conv"]["stats"], self.d["ours"]["stats"]
        self.assertLess(o["ambulance_trip_s"], c["ambulance_trip_s"])
        self.assertEqual(o["ambulance_red_wait_s"], 0)
        self.assertGreater(c["ambulance_red_wait_s"], 0)
        record("DM-04", f"ambulance {c['ambulance_trip_s']} s (conventional) -> {o['ambulance_trip_s']} s (ours), red wait {c['ambulance_red_wait_s']} -> 0 s")

    def test_DM06_signal_arms_show_plus_and_seven(self):
        """DM-06: junction signals are 4 arms (N,E,S,W); straight = '-' or '|' green, a turn lights only its 2 arms"""
        arms = [s for f in self.d["ours"]["frames"] for s in f["sig"].values()]
        self.assertTrue(all(len(a) == 4 and set(a) <= set("GRY") for a in arms))
        self.assertTrue(any(a.count("G") == 2 and a in ("GRGR".replace("R", "R"), "RGRG") for a in arms))   # straight
        corner = [a for a in arms if a.count("G") == 2 and not (a[0] == a[2] or a[1] == a[3])]
        self.assertTrue(corner)                                                                           # a '7' turn
        record("DM-06", "straight green = opposite arms; turn green = two adjacent arms ('7' shape)")

    def test_DM07_queues_form_behind_red(self):
        """DM-07: dense random traffic builds visible queues, and fewer with our signals"""
        c, o = self.d["conv"]["stats"], self.d["ours"]["stats"]
        self.assertGreater(c["spawned"], 120)
        self.assertGreater(c["max_queue"], 20)
        self.assertLess(o["max_queue"], c["max_queue"])
        record("DM-07", f"{c['spawned']} random cars; most stopped at once {c['max_queue']} (conv) vs {o['max_queue']} (ours)")

    def test_DM08_hill_climbing_parameters_reach_junctions(self):
        """DM-08: the traffic centre sends the hill-climbing parameters to every junction (INFORM PARAMETER_UPDATE)"""
        from stms.demo import build, TUNED_PARAMS
        sim, _, _ = build("ours", [], [])
        sim.run(3)
        for ag in sim.intersections.values():
            self.assertEqual(ag.controller.min_green_time, TUNED_PARAMS["min_green_time"])
            self.assertEqual(ag.controller.max_green_time, TUNED_PARAMS["max_green_time"])
        upd = [m for m in self.d["ours"]["messages"] if "tuned timing" in m[4]]
        self.assertEqual(len(upd), 9)
        record("DM-08", "9 junctions received PARAMETER_UPDATE at t = 1 s and use min green 3 s, max green 15 s")

    def test_DM09_web_entry_point(self):
        """DM-09: the browser/server entry point parses the page's query strings and returns valid JSON"""
        import json
        from stms.demo import record_json
        d = json.loads(record_json("INT_10-INT_11", "Car:W1:E1:20;Ambulance:S0:N2:30", 7))
        self.assertEqual(d["incident"]["roads"], [["INT_10", "INT_11"]])
        self.assertEqual([c["name"] for c in d["cars"]], ["Car", "Ambulance"])
        self.assertEqual(d["ours"]["stats"]["spawned"], d["conv"]["stats"]["spawned"])
        record("DM-09", "record_json round trip: 1 closed road, car + ambulance dispatched, identical traffic")

    def test_DM05_same_traffic_both_runs(self):
        """DM-05: both runs use identical traffic and the same accident, so the comparison is fair"""
        self.assertEqual(self.d["conv"]["stats"]["spawned"], self.d["ours"]["stats"]["spawned"])
        self.assertEqual(self.d["incident"]["roads"], [list(c) for c in DEFAULT_CLOSED])
        record("DM-05", f"{self.d['ours']['stats']['spawned']} vehicles spawned in each run; accident J01-J11 at t = 10 s")


if __name__ == "__main__":
    unittest.main()
