"""
The demo: ONE city, run twice with identical traffic and identical accidents.

    CONVENTIONAL : fixed-time signals; drivers only discover a closed road when they reach it;
                   the ambulance waits at red lights like everyone else.
    OURS (STMS)  : Max-Pressure signals; the traffic centre broadcasts the accident and cars re-plan with A*;
                   the ambulance asks junctions for a green corridor (preemption).

3 x 3 city with DENSE RANDOM background traffic (nobody in it is tracked, no ambulances). On top of that the
user can DISPATCH one Car and an Ambulance: each starts at a random boundary road and drives to a random
destination along its A* (shortest) route. record() returns plain data for the page: map, per-frame positions,
per-junction "+" signal arms, queue counts, each dispatched vehicle's original / current route and a results table.
Owner: Member 1 (scenario) + Member 3 (vehicle re-planning) + Member 4 (ambulance)
"""

import random
from typing import Any, Dict, List, Optional

from src import scenarios as S
from src.algorithms.max_pressure import axis_of

INCIDENT_AT = 10
DEFAULT_CLOSED = [("INT_01", "INT_11")]       # roads closed in the default demo (both directions)
INCIDENT_ROAD = DEFAULT_CLOSED[0]            # kept for the tests
DURATION = 240
RATE = 1.1                                   # background cars per second (dense)
TRAFFIC_SECONDS = 150
COLOURS = {"Car 1": "#2563eb", "Ambulance": "#dc2626"}
NAMES = ["Car 1", "Ambulance"]
# a fixed example used by the offline page and the tests: (name, origin, destination, departure second)
SAMPLE_DISPATCH = [("Car 1", "N1", "S1", 15), ("Ambulance", "S0", "N2", 50)]
J = lambda n: "J" + n[4:]
N, E, SOUTH, W = 0, 1, 2, 3                   # arm order of the "+" signal


def _arm_index(net) -> Dict[tuple, int]:
    """(junction, neighbour) -> which arm of the '+' the neighbour is on (0=N 1=E 2=S 3=W)."""
    idx = {}
    for seg in net.segments.values():
        for a, b in ((seg.source, seg.target), (seg.target, seg.source)):
            if a.startswith("INT"):
                (x1, y1), (x2, y2) = net.node_positions[a], net.node_positions[b]
                dx, dy = x2 - x1, y2 - y1
                idx[(a, b)] = (N if dy > 0 else SOUTH) if abs(dy) > abs(dx) else (E if dx > 0 else W)
    return idx


def _heading(net, v) -> int:
    seg = v.current_segment
    if seg is None:
        return 0
    (x1, y1), (x2, y2) = net.node_positions[seg.source], net.node_positions[seg.target]
    dx, dy = x2 - x1, y2 - y1
    return (1 if dy > 0 else 3) if abs(dy) > abs(dx) else (0 if dx > 0 else 2)


def _signals(sim, armidx, vmap):
    """Per junction: arm colours 'NESW' as e.g. 'RGRG', and the number of stopped cars waiting on each arm.
    Green arm = a road whose cars may go. The first car in each green approach decides which exit arm is
    lit too, so left-to-right shows '-' green, and left-to-down shows a '7' (left + down arms green)."""
    net = sim.network
    sig, queue = {}, {}
    for nid, ag in sim.intersections.items():
        ns, ew = ag.ns_incoming_roads, ag.ew_incoming_roads
        arms, qq = ["R"] * 4, [0] * 4
        for rid in ns + ew:
            seg = net.segment_by_id(rid)
            a = armidx[(nid, seg.source)]
            qq[a] = sum(1 for vid in seg.vehicles if vid in vmap and vmap[vid].current_speed < 0.5)
        ax = axis_of(ag.current_phase)
        if ax is None:                                                     # yellow: the arms that were green
            for rid in (ns if axis_of(ag.previous_green) == "NS" else ew):
                arms[armidx[(nid, net.segment_by_id(rid).source)]] = "Y"
        else:
            roads, lit = (ns if ax == "NS" else ew), set()
            for rid in roads:
                seg = net.segment_by_id(rid)
                a = armidx[(nid, seg.source)]
                if seg.vehicles:
                    head = vmap.get(seg.vehicles[0])
                    nxt = head._next_segment() if head is not None and head.current_segment is seg else None
                    lit |= {a, armidx[(nid, nxt.target)] if nxt is not None else (a + 2) % 4}
            if not lit:                                                    # nobody waiting: show the whole axis
                lit = {armidx[(nid, net.segment_by_id(r).source)] for r in roads}
            for a in lit:
                arms[a] = "G"
        sig[nid], queue[nid] = "".join(arms), qq
    return sig, queue


def build(variant="ours", closed=None, dispatch=None, seed=5):
    """variant: 'ours' or 'conventional'. Returns (sim, dispatched vehicles by name, their ids)."""
    closed = DEFAULT_CLOSED if closed is None else [tuple(c) for c in closed]
    dispatch = SAMPLE_DISPATCH if dispatch is None else dispatch
    ours = variant == "ours"
    sim = S.make_sim(3, 3, mode="MAX_PRESSURE" if ours else "FIXED_TIME",
                     reroute_policy="broadcast" if ours else "local", green_wave=ours)
    sim.spawn_poisson_demand(S.boundary_od_pairs(3, 3), RATE, TRAFFIC_SECONDS, random.Random(seed))   # random crowd
    cars = {}
    for name, o, d, t in dispatch:
        if name == "Ambulance":
            v = sim.spawn_emergency_vehicle(o, d, departure_time=t, preemption_enabled=ours)
            if not ours:
                v.reroute_policy = "local"
        else:
            v = sim.spawn_vehicle(o, d, departure_time=t)
        cars[name] = v

    def accident(s):
        for a, b in closed:                      # a closed road is closed in both directions
            s.coordinator.report_incident(a, b)
            s.coordinator.report_incident(b, a)
    if closed:
        sim.schedule(INCIDENT_AT, accident)
    return sim, cars, {v.agent_id for v in cars.values()}


def _run(variant, closed, dispatch=None, duration=DURATION, seed=5):
    dispatch = SAMPLE_DISPATCH if dispatch is None else dispatch
    both = {c for a, b in closed for c in ((a, b), (b, a))}
    shut = ", ".join(f"{J(a)}-{J(b)}" for a, b in closed)
    sim, cars, ids = build(variant, closed, dispatch, seed)
    net, armidx = sim.network, _arm_index(sim.network)
    original = {n: list(v.planned_path) for n, v in cars.items()}
    last = dict(original)
    frames, events = [], []
    for _ in range(duration):
        sim.step()
        vmap = {v.agent_id: v for v in sim.vehicles if v.departed and not v.has_arrived}
        sig, queue = _signals(sim, armidx, vmap)
        frame = {"sig": sig, "q": queue, "cars": {}, "bg": [], "wait": 0}
        for n, v in cars.items():
            if v.agent_id in vmap:
                x, y = sim._drawn_xy(v)
                frame["cars"][n] = [round(x), round(y), 1 if v.current_speed < 0.5 else 0, _heading(net, v)]
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
        for vid, v in vmap.items():
            stopped = 1 if v.current_speed < 0.5 and v.current_segment is not None else 0
            frame["wait"] += stopped
            if vid not in ids:
                x, y = sim._drawn_xy(v)
                frame["bg"].append([round(x), round(y), stopped, _heading(net, v)])
        frames.append(frame)
    summ = sim.summary()
    arrived = {n: (v.arrival_time if v.has_arrived else None) for n, v in cars.items()}
    dep = {n: t for n, o, d, t in dispatch}
    amb = cars.get("Ambulance")
    return {"frames": frames, "events": events, "arrived": arrived, "depart": dep,
            "stats": {"trip": {n: (v.arrival_time - dep[n] if v.has_arrived else None) for n, v in cars.items()},
                      "wait": {n: round(v.total_wait_time) for n, v in cars.items()},
                      "ambulance_trip_s": (amb.arrival_time - amb.departure_time) if amb is not None and amb.has_arrived else None,
                      "ambulance_red_wait_s": amb.total_wait_time if amb is not None else None,
                      "avg_wait_s": summ["avg_wait_s"], "arrived": summ["arrived"], "spawned": summ["spawned"],
                      "max_queue": max((f["wait"] for f in frames), default=0),
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


def record(closed=None, dispatch: Optional[list] = None, seed: int = 5) -> Dict[str, Any]:
    """dispatch: list of (name, origin, destination, departure second); None = SAMPLE_DISPATCH, [] = nobody."""
    closed = DEFAULT_CLOSED if closed is None else [tuple(c) for c in closed]
    dispatch = SAMPLE_DISPATCH if dispatch is None else [tuple(d) for d in dispatch]
    ours, original = _run("ours", closed, dispatch, seed=seed)
    conv, _ = _run("conventional", closed, dispatch, seed=seed)
    net = S.make_sim(3, 3).network
    return {
        "nodes": {k: [round(p[0]), round(p[1])] for k, p in net.node_positions.items()},
        "roads": [[s.source, s.target] for s in net.segments.values()],
        "signals": [k for k in net.node_positions if k.startswith("INT")],
        "incident": {"t": INCIDENT_AT, "roads": [list(c) for c in closed]},
        "cars": [{"name": n, "from": o, "to": d, "t": t, "original": original[n], "colour": COLOURS[n]}
                 for n, o, d, t in dispatch],
        "seed": seed, "ours": ours, "conv": conv, "frames": DURATION,
    }
