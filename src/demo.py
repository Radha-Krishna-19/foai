"""
The demo: ONE scenario, run twice with identical traffic and the identical accident.

    CONVENTIONAL : fixed-time signals; drivers only discover a closed road when they reach it;
                   the ambulance waits at red lights like everyone else.
    OURS (STMS)  : Max-Pressure signals; the traffic centre broadcasts the accident and cars re-plan with A*;
                   the ambulance asks junctions for a green corridor (preemption).

3 x 3 city, 6 tracked cars + 1 ambulance + background traffic. At t = INCIDENT_AT the chosen roads close
(both directions). record() returns plain data for the page: map, per-frame positions of both runs, each
car's ORIGINAL route, its CURRENT route, the re-route events (with a reason) and a results table.
Owner: Member 1 (scenario) + Member 3 (vehicle re-planning) + Member 4 (ambulance)
"""

import random
from typing import Any, Dict, List

from src import scenarios as S

INCIDENT_AT = 10
DEFAULT_CLOSED = [("INT_01", "INT_11")]       # roads closed in the default demo (both directions)
INCIDENT_ROAD = DEFAULT_CLOSED[0]            # kept for the tests
DURATION = 190
COLOURS = ["#2563eb", "#16a34a", "#9333ea", "#ea580c", "#0891b2", "#db2777"]
AMBULANCE = ("S0", "N2", 14)
# name: (origin, destination, departure second)
TRACKED = {"Car 1": ("N1", "S1", 0), "Car 2": ("W1", "E1", 0), "Car 3": ("N0", "S2", 0),
           "Car 4": ("W0", "E2", 2), "Car 5": ("S1", "N1", 2), "Car 6": ("E2", "W0", 4)}
J = lambda n: "J" + n[4:]


def build(variant="ours", closed=None, seed=5):
    """variant: 'ours' or 'conventional'. Returns (sim, cars incl. 'Ambulance', ids of tracked vehicles)."""
    closed = DEFAULT_CLOSED if closed is None else [tuple(c) for c in closed]
    ours = variant == "ours"
    sim = S.make_sim(3, 3, mode="MAX_PRESSURE" if ours else "FIXED_TIME",
                     reroute_policy="broadcast" if ours else "local", green_wave=ours)
    cars = {n: sim.spawn_vehicle(o, d, departure_time=t) for n, (o, d, t) in TRACKED.items()}
    o, d, t = AMBULANCE
    amb = sim.spawn_emergency_vehicle(o, d, departure_time=t, preemption_enabled=ours)
    if not ours:
        amb.reroute_policy = "local"
    cars["Ambulance"] = amb
    ids = {v.agent_id for v in cars.values()}
    sim.spawn_poisson_demand(S.boundary_od_pairs(3, 3), 0.3, 60, random.Random(seed))     # background traffic

    def accident(s):
        for a, b in closed:                      # a closed road is closed in both directions
            s.coordinator.report_incident(a, b)
            s.coordinator.report_incident(b, a)
    if closed:
        sim.schedule(INCIDENT_AT, accident)
    return sim, cars, ids


def _run(variant, closed, duration=DURATION, seed=5):
    both = {c for a, b in closed for c in ((a, b), (b, a))}
    shut = ", ".join(f"{J(a)}-{J(b)}" for a, b in closed)
    sim, cars, ids = build(variant, closed, seed)
    original = {n: list(v.planned_path) for n, v in cars.items()}
    last = dict(original)
    frames, events = [], []
    for _ in range(duration):
        sim.step()
        frame = {"light": {k: a.current_phase.value for k, a in sim.intersections.items()}, "cars": {}, "bg": []}
        for n, v in cars.items():
            if v.departed and not v.has_arrived:
                x, y = sim._drawn_xy(v)
                frame["cars"][n] = [round(x), round(y), 1 if v.current_speed < 0.5 else 0]
                if v.planned_path != last[n]:
                    old = last[n]
                    hit = any((old[i], old[i + 1]) in both for i in range(len(old) - 1))
                    if variant == "ours":
                        why = (f"traffic centre alert: closed road ({shut}) was on its route" if hit
                               else "traffic centre alert: closed road ahead")
                    else:
                        why = "saw the closed road at the next junction and had to find another way"
                    events.append({"t": int(sim.sim_time), "car": n, "old": old, "new": list(v.planned_path), "why": why})
                    last[n] = list(v.planned_path)
        for v in sim.vehicles:
            if v.agent_id not in ids and v.departed and not v.has_arrived:
                x, y = sim._drawn_xy(v)
                frame["bg"].append([round(x), round(y)])
        frames.append(frame)
    amb = cars["Ambulance"]
    summ = sim.summary()
    arrived = {n: (v.arrival_time if v.has_arrived else None) for n, v in cars.items()}
    return {"frames": frames, "events": events, "arrived": arrived, "depart": AMBULANCE[2],
            "stats": {"ambulance_trip_s": (amb.arrival_time - amb.departure_time) if amb.has_arrived else None,
                      "ambulance_red_wait_s": amb.total_wait_time,
                      "avg_wait_s": summ["avg_wait_s"], "arrived": summ["arrived"], "spawned": summ["spawned"],
                      "rerouted": len({e["car"] for e in events if e["car"] != "Ambulance"})}}, original


def inner_roads() -> List[tuple]:
    """The 12 two-way roads between junctions of the 3x3 city (the ones an accident can close)."""
    net = S.make_sim(3, 3).network
    seen, out = set(), []
    for seg in net.segments.values():
        a, b = seg.source, seg.target
        if a.startswith("INT") and b.startswith("INT") and (b, a) not in seen:
            seen.add((a, b))
            out.append((a, b))
    return out


def record_all_single() -> Dict[str, Any]:
    """Pre-computed runs for 'no accident' and for each single closed road (used by the offline demo.html)."""
    out = {"none": record([])}
    for a, b in inner_roads():
        out[f"{a}-{b}"] = record([(a, b)])
    return out


def record(closed=None) -> Dict[str, Any]:
    closed = DEFAULT_CLOSED if closed is None else [tuple(c) for c in closed]
    ours, original = _run("ours", closed)
    conv, _ = _run("conventional", closed)
    net = S.make_sim(3, 3).network
    names = list(TRACKED) + ["Ambulance"]
    meta = {**{n: (TRACKED[n][0], TRACKED[n][1]) for n in TRACKED}, "Ambulance": AMBULANCE[:2]}
    return {
        "nodes": {k: [round(p[0]), round(p[1])] for k, p in net.node_positions.items()},
        "roads": [[s.source, s.target] for s in net.segments.values()],
        "signals": [k for k in net.node_positions if k.startswith("INT")],
        "incident": {"t": INCIDENT_AT, "roads": [list(c) for c in closed]},
        "cars": [{"name": n, "from": meta[n][0], "to": meta[n][1], "original": original[n],
                  "colour": "#dc2626" if n == "Ambulance" else COLOURS[i]} for i, n in enumerate(names)],
        "ours": ours, "conv": conv, "frames": DURATION,
    }
