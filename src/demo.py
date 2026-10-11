"""
The demo: ONE scenario that shows how cars re-plan.

    3 x 3 city, 6 tracked cars + background traffic, every junction run by a Max-Pressure agent.
    At t = INCIDENT_AT an accident closes the road INT_01 <-> INT_11 (both directions).
    The coordinator broadcasts INCIDENT_ALERT; each car checks "is the closed road on MY route?".
    If yes it re-runs A* with its updated belief and picks a new route.

record() returns plain data (map, per-frame positions, each car's ORIGINAL route, CURRENT route and the
re-route events with a one-line reason). `python run_demo.py` turns it into a single HTML page.
Owner: Member 1 (scenario) + Member 3 (vehicle re-planning)
"""

import random
from typing import Any, Dict, List

from src import scenarios as S

INCIDENT_AT = 10
DEFAULT_CLOSED = [("INT_01", "INT_11")]       # roads closed in the default demo (both directions)
INCIDENT_ROAD = DEFAULT_CLOSED[0]            # kept for the tests
DURATION = 130
COLOURS = ["#2563eb", "#16a34a", "#9333ea", "#ea580c", "#0891b2", "#db2777"]
# name: (origin, destination, departure second)
TRACKED = {"Car 1": ("N1", "S1", 0), "Car 2": ("W1", "E1", 0), "Car 3": ("N0", "S2", 0),
           "Car 4": ("W0", "E2", 2), "Car 5": ("S1", "N1", 2), "Car 6": ("E2", "W0", 4)}


def build(closed=None):
    closed = DEFAULT_CLOSED if closed is None else [tuple(c) for c in closed]
    sim = S.make_sim(3, 3)
    cars = {}
    for name, (o, d, dep) in TRACKED.items():
        cars[name] = sim.spawn_vehicle(o, d, departure_time=dep)
    tracked_ids = {v.agent_id for v in cars.values()}
    sim.spawn_poisson_demand(S.boundary_od_pairs(3, 3), 0.15, 60, random.Random(5))   # background traffic
    def accident(s):
        for a, b in closed:                      # a closed road is closed in both directions
            s.coordinator.report_incident(a, b)
            s.coordinator.report_incident(b, a)
    if closed:
        sim.schedule(INCIDENT_AT, accident)
    return sim, cars, tracked_ids


def _route(v) -> List[str]:
    """Route still to be driven, starting at the node the car is heading to / standing on."""
    start = v.path_index + (1 if v.current_segment is not None else 0)
    return list(v.planned_path[max(0, start - (1 if v.current_segment is not None else 0)):])


def record(closed=None) -> Dict[str, Any]:
    closed = DEFAULT_CLOSED if closed is None else [tuple(c) for c in closed]
    both = {c for a, b in closed for c in ((a, b), (b, a))}
    sim, cars, tracked_ids = build(closed)
    net = sim.network
    original = {n: list(v.planned_path) for n, v in cars.items()}
    last_path = dict(original)
    frames, events = [], []
    for _ in range(DURATION):
        sim.step()
        t = int(sim.sim_time)
        frame = {"t": t, "light": {k: a.current_phase.value for k, a in sim.intersections.items()},
                 "cars": {}, "bg": []}
        for n, v in cars.items():
            if v.departed and not v.has_arrived:
                x, y = sim._drawn_xy(v)
                frame["cars"][n] = [round(x), round(y), 1 if v.current_speed < 0.5 else 0, _route(v)]
                if v.planned_path != last_path[n]:
                    old = last_path[n]
                    hit = any((old[i], old[i + 1]) in both for i in range(len(old) - 1))
                    J = lambda n: "J" + n[4:]
                    shut = ", ".join(f"{J(a)}-{J(b)}" for a, b in closed)
                    why = (f"a closed road ({shut}) was on the car's route"
                           if hit else "a closed road was ahead on its route")
                    events.append({"t": t, "car": n, "old": old, "new": list(v.planned_path), "why": why})
                    last_path[n] = list(v.planned_path)
        for v in sim.vehicles:
            if v.agent_id not in tracked_ids and v.departed and not v.has_arrived:
                x, y = sim._drawn_xy(v)
                frame["bg"].append([round(x), round(y)])
        frames.append(frame)
    return {
        "nodes": {k: [round(p[0]), round(p[1])] for k, p in net.node_positions.items()},
        "roads": [[s.source, s.target] for s in net.segments.values()],
        "signals": list(sim.intersections),
        "incident": {"t": INCIDENT_AT, "roads": [list(c) for c in closed]},
        "cars": [{"name": n, "from": o, "to": d, "colour": COLOURS[i], "original": original[n]}
                 for i, (n, (o, d, _)) in enumerate(TRACKED.items())],
        "events": events, "frames": frames,
        "arrived": {n: cars[n].arrival_time for n in cars},
    }
