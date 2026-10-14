"""
Demo recorder - runs one city twice with identical traffic and records everything the web page draws.

    Fixed-time city (baseline) : fixed-time signals, drivers only discover a closed road when they reach it,
                                 the ambulance waits at red lights like everyone else.
    STMS city                  : Max-Pressure signals with hill-climbing-tuned parameters, the traffic centre
                                 broadcasts accidents so vehicles re-plan with A*, the ambulance requests a
                                 green corridor (FIPA-ACL preemption).

The page can dispatch one Car and one Ambulance (random origin -> random destination) and close any inner road.
record() returns plain JSON-serialisable data; the same function runs on the local server (run_demo.py) and
inside the browser (Pyodide), so the website always shows the real Python agents.
"""

import random
from typing import Any, Dict, List, Optional

from stms import scenarios as S
from stms.algorithms.max_pressure import axis_of
from stms.algorithms.search_astar import AStarRouter
from stms.protocols.message import Performative

ROWS = COLS = 3
GRID = dict(feeder_length=120.0, feeder_capacity=16)   # short boundary roads so the city fills the map
INCIDENT_AT = 10
DEFAULT_CLOSED = [("INT_01", "INT_11")]
INCIDENT_ROAD = DEFAULT_CLOSED[0]
DURATION = 240
RATE = 1.1                     # background vehicles per second (Poisson)
TRAFFIC_SECONDS = 150
NAMES = ["Car", "Ambulance"]
SAMPLE_DISPATCH = [("Car", "N1", "S1", 15), ("Ambulance", "W0", "E2", 40)]

# Output of experiment S6 (hill climbing with random restarts over the Max-Pressure parameters).
# Regenerate with: python scripts/generate_results.py
TUNED_PARAMS = {"min_green_time": 3.0, "max_green_time": 15.0, "starvation_weight": 1.5, "switch_threshold": 3.0}

N, E, SOUTH, W = 0, 1, 2, 3    # arm order of the "+" signal

# readable form of the junction agent's decision reason (index is stored per frame)
REASONS = [("yellow clearance complete", "yellow finished, switching"),
           ("yellow clearance", "yellow, clearing the junction"),
           ("min-green hold", "holding the minimum green"),
           ("no demand on red approach", "keeps green, no car is waiting on the red side"),
           ("max-green reached", "maximum green reached, switching"),
           ("higher pressure on red approach", "red side has higher pressure, switching"),
           ("current phase has higher pressure", "green side has higher pressure"),
           ("EMERGENCY", "ambulance priority (preemption)"),
           ("fixed-time cycle", "fixed 20 s cycle")]


def _reason_code(text: str) -> int:
    for i, (k, _) in enumerate(REASONS):
        if text.startswith(k):
            return i
    return len(REASONS) - 1


def J(n: str) -> str:
    return "J" + n[4:] if n.startswith("INT_") else n


def make_city(variant: str):
    ours = variant == "ours"
    return S.make_sim(ROWS, COLS, mode="MAX_PRESSURE" if ours else "FIXED_TIME",
                      reroute_policy="broadcast" if ours else "local", green_wave=ours, grid_kw=GRID)


def _arm_index(net) -> Dict[tuple, int]:
    """(junction, neighbour) -> arm of the '+' the neighbour lies on (0=N 1=E 2=S 3=W)."""
    idx = {}
    for seg in net.segments.values():
        for a, b in ((seg.source, seg.target), (seg.target, seg.source)):
            if a.startswith("INT"):
                (x1, y1), (x2, y2) = net.node_positions[a], net.node_positions[b]
                dx, dy = x2 - x1, y2 - y1
                idx[(a, b)] = (N if dy > 0 else SOUTH) if abs(dy) > abs(dx) else (E if dx > 0 else W)
    return idx


def _heading(net, v) -> int:
    """0 = east, 1 = north, 2 = west, 3 = south."""
    seg = v.current_segment
    if seg is None:
        return 0
    (x1, y1), (x2, y2) = net.node_positions[seg.source], net.node_positions[seg.target]
    dx, dy = x2 - x1, y2 - y1
    return (1 if dy > 0 else 3) if abs(dy) > abs(dx) else (0 if dx > 0 else 2)


def _signals(sim, armidx, vmap):
    """Per junction: arm colours in N,E,S,W order (e.g. 'RGRG'), stopped cars per arm and Max-Pressure scores.
    A straight movement lights two opposite arms ('-' or '|'); a turning lead car lights its entry and exit
    arms (a '7' shape)."""
    net = sim.network
    sig, queue, press = {}, {}, {}
    for nid, ag in sim.intersections.items():
        ns, ew = ag.ns_incoming_roads, ag.ew_incoming_roads
        arms, qq = ["R"] * 4, [0] * 4
        for rid in ns + ew:
            seg = net.segment_by_id(rid)
            qq[armidx[(nid, seg.source)]] = sum(1 for vid in seg.vehicles
                                                if vid in vmap and vmap[vid].current_speed < 0.5)
        ax = axis_of(ag.current_phase)
        if ax is None:
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
            if not lit:
                lit = {armidx[(nid, net.segment_by_id(r).source)] for r in roads}
            for a in lit:
                arms[a] = "G"
        d = ag.last_decision
        sig[nid], queue[nid] = "".join(arms), qq
        press[nid] = ([round(d.ns_score, 1), round(d.ew_score, 1), ag.ns_in_queue, ag.ew_in_queue,
                       _reason_code(d.reason)] if d is not None else [0, 0, 0, 0, 2])
    return sig, queue, press


def build(variant="ours", closed=None, dispatch=None, seed=5):
    """Returns (sim, dispatched vehicles by name, their agent ids)."""
    closed = DEFAULT_CLOSED if closed is None else [tuple(c) for c in closed]
    dispatch = SAMPLE_DISPATCH if dispatch is None else dispatch
    ours = variant == "ours"
    sim = make_city(variant)
    sim.spawn_poisson_demand(S.boundary_od_pairs(ROWS, COLS), RATE, TRAFFIC_SECONDS, random.Random(seed))
    cars = {}
    for name, o, d, t in dispatch:
        if name == "Ambulance":
            v = sim.spawn_emergency_vehicle(o, d, departure_time=t, preemption_enabled=ours)
            if not ours:
                v.reroute_policy = "local"
        else:
            v = sim.spawn_vehicle(o, d, departure_time=t)
        cars[name] = v

    if ours:      # the traffic centre pushes the hill-climbing result to every junction (INFORM PARAMETER_UPDATE)
        def push_params(s):
            for iid in s.intersections:
                s.coordinator.send_message(iid, Performative.INFORM, {"type": "PARAMETER_UPDATE",
                                                                      "params": TUNED_PARAMS},
                                           ontology="policy-tuning")
        sim.schedule(1, push_params)

    def accident(s):
        for a, b in closed:
            if ours:                              # the traffic centre detects it and broadcasts an alert
                s.coordinator.report_incident(a, b)
                s.coordinator.report_incident(b, a)
            else:                                 # baseline city: the road is simply closed, nobody is told
                s.network.set_incident(a, b, True)
                s.network.set_incident(b, a, True)
    if closed:
        sim.schedule(INCIDENT_AT, accident)
    return sim, cars, {v.agent_id for v in cars.values()}


def _message_log(sim, names: Dict[str, str]) -> List[list]:
    """Agent messages as [t, sender, act, receiver, text] with readable names."""
    def nm(a):
        if a in names:
            return names[a]
        if a.startswith("TMC"):
            return "Traffic centre"
        return "All agents" if a == "BROADCAST" else J(a)
    out = []
    for m in sim.broker.history():
        c = m.content
        kind = c.get("type") or c.get("event") or ""
        if kind == "INCIDENT_ALERT":
            seg = c.get("blocked_segment") or c.get("segment") or ["?", "?"]
            text = f"road {J(seg[0])}-{J(seg[1])} closed, re-plan with A*"
        elif m.performative == Performative.PREEMPT_OVERRIDE:
            text = f"request green ({c.get('corridor_direction')})"
        elif m.performative == Performative.AGREE:
            text = "green granted"
        elif m.performative == Performative.CANCEL:
            text = "passed, resume normal control"
        elif kind == "PLATOON_INCOMING":
            text = f"{c.get('estimated_vehicles')} cars coming ({c.get('axis')})"
        elif kind == "PARAMETER_UPDATE":
            p = c.get("params", {})
            text = f"use tuned timing: min green {p.get('min_green_time'):g} s, max green {p.get('max_green_time'):g} s"
        else:
            text = kind.lower().replace("_", " ")
        out.append([int(m.timestamp), nm(m.sender_id), m.performative.value, nm(m.receiver_id), text])
    return out


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
        sig, queue, press = _signals(sim, armidx, vmap)
        frame = {"sig": sig, "q": queue, "p": press, "cars": {}, "bg": [], "wait": 0}
        for n, v in cars.items():
            if v.agent_id in vmap:
                x, y = sim._drawn_xy(v, 7.0)
                frame["cars"][n] = [round(x), round(y), 1 if v.current_speed < 0.5 else 0, _heading(net, v)]
            if v.planned_path != last[n]:
                old = last[n]
                hit = any((old[i], old[i + 1]) in both for i in range(len(old) - 1))
                if variant == "ours":
                    why = f"accident alert from the traffic centre ({shut})" if hit else "accident alert"
                else:
                    why = "found the closed road at the junction"
                events.append({"t": int(sim.sim_time), "car": n, "old": old, "new": list(v.planned_path), "why": why})
                last[n] = list(v.planned_path)
        for vid, v in vmap.items():
            stopped = 1 if v.current_speed < 0.5 and v.current_segment is not None else 0
            frame["wait"] += stopped
            if vid not in ids:
                x, y = sim._drawn_xy(v, 7.0)
                frame["bg"].append([round(x), round(y), stopped, _heading(net, v)])
        frames.append(frame)
    summ = sim.summary()
    dep = {n: t for n, o, d, t in dispatch}
    amb = cars.get("Ambulance")
    names = {v.agent_id: n for n, v in cars.items()}
    return {"frames": frames, "events": events, "messages": _message_log(sim, names),
            "arrived": {n: (v.arrival_time if v.has_arrived else None) for n, v in cars.items()},
            "depart": dep,
            "stats": {"trip": {n: (v.arrival_time - dep[n] if v.has_arrived else None) for n, v in cars.items()},
                      "wait": {n: round(v.total_wait_time) for n, v in cars.items()},
                      "replans": {n: v.plan_count for n, v in cars.items()},
                      "ambulance_trip_s": (amb.arrival_time - amb.departure_time) if amb is not None and amb.has_arrived else None,
                      "ambulance_red_wait_s": amb.total_wait_time if amb is not None else None,
                      "avg_wait_s": summ["avg_wait_s"], "arrived": summ["arrived"], "spawned": summ["spawned"],
                      "max_queue": max((f["wait"] for f in frames), default=0),
                      "messages": sim.broker.total_messages_dispatched,
                      "rerouted": len({e["car"] for e in events if e["car"] != "Ambulance"})}}, original


def astar_plan(origin: str, destination: str) -> Dict[str, Any]:
    """The vehicle agent's first A* plan on the open network: nodes expanded and route cost (seconds)."""
    r = AStarRouter(make_city("ours").network).search(origin, destination)
    return {"expanded": r.nodes_expanded, "cost": round(r.cost, 1), "path": r.path}


def inner_roads() -> List[tuple]:
    net = make_city("ours").network
    seen, out = set(), []
    for seg in net.segments.values():
        a, b = seg.source, seg.target
        if a.startswith("INT") and b.startswith("INT") and (b, a) not in seen:
            seen.add((a, b))
            out.append((a, b))
    return out


def record(closed=None, dispatch: Optional[list] = None, seed: int = 5) -> Dict[str, Any]:
    """dispatch: list of (name, origin, destination, departure second); None = SAMPLE_DISPATCH, [] = none."""
    closed = DEFAULT_CLOSED if closed is None else [tuple(c) for c in closed]
    dispatch = SAMPLE_DISPATCH if dispatch is None else [tuple(d) for d in dispatch]
    ours, original = _run("ours", closed, dispatch, seed=seed)
    conv, _ = _run("conventional", closed, dispatch, seed=seed)
    net = make_city("ours").network
    return {
        "nodes": {k: [round(p[0]), round(p[1])] for k, p in net.node_positions.items()},
        "roads": [[s.source, s.target] for s in net.segments.values()],
        "signals": [k for k in net.node_positions if k.startswith("INT")],
        "incident": {"t": INCIDENT_AT, "roads": [list(c) for c in closed]},
        "cars": [{"name": n, "from": o, "to": d, "t": t, "original": original[n],
                  "astar": astar_plan(o, d)} for n, o, d, t in dispatch],
        "reasons": [r for _, r in REASONS], "tuned": TUNED_PARAMS, "seed": seed, "ours": ours, "conv": conv, "frames": DURATION,
    }


def record_json(closed_text: str = "", dispatch_text: str = "", seed: int = 5) -> str:
    """Strings in, JSON out - the entry point used by the browser (Pyodide) and the local server.
    closed_text: 'INT_01-INT_11,INT_10-INT_11'   dispatch_text: 'Car:N0:S2:35;Ambulance:W1:E0:50'"""
    import json
    closed = []
    for part in filter(None, closed_text.split(",")):
        a, _, b = part.partition("-")
        if a.startswith("INT_") and b.startswith("INT_"):
            closed.append((a, b))
    disp = []
    for part in filter(None, dispatch_text.split(";")):
        n, o, d, t = part.split(":")
        if n in NAMES and o != d:
            disp.append((n, o, d, int(float(t))))
    return json.dumps(record(closed, disp, int(seed)), separators=(",", ":"))
