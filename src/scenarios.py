"""
Evaluation scenarios (used by the demo runner, the test-suite and the report generator).

Every scenario is deterministic for its seeds, runs every vehicle to completion
(run_until_clear), and averages over several random seeds where randomness matters.

    S1  Fixed-Time vs Max-Pressure (+ green-wave ablation)            Member 2
    S2  Rush-hour surge & anti-starvation ablation                     Member 2
    S3  Emergency green corridor (same controller, preemption on/off)  Member 4
    S4  Accident: static vs local vs broadcast re-routing              Member 3 + 5
    S5  Bus Transit Signal Priority negotiation                        Member 4
    S6  Hill-climbing policy tuning (train vs validation seeds)        Member 5
    S7  Search algorithm comparison (BFS/UCS/Greedy/A*/W-A*)           Member 3
    S8  Scalability 2x2 .. 8x8 grids                                   Member 1
"""

import random
import statistics as st
import time
from typing import Any, Dict, List, Optional

from src.protocols.message import MessageBroker, Performative
from src.environment.traffic_grid import build_grid_network
from src.environment.simulator import TrafficSimulator
from src.algorithms.search_astar import AStarRouter
from src.agents.intersection_agent import DEFAULT_PARAMS

OD_2x2 = [("N0", "S0"), ("N1", "S1"), ("W0", "E0"), ("W1", "E1"), ("S0", "N0"), ("S1", "N1"),
          ("E0", "W0"), ("E1", "W1"), ("N0", "E1"), ("W0", "S1"), ("S1", "W0"), ("E0", "S0")]


def _mean(xs):
    xs = [x for x in xs if x is not None]
    return round(st.mean(xs), 2) if xs else None


def _std(xs):
    xs = [x for x in xs if x is not None]
    return round(st.stdev(xs), 2) if len(xs) > 1 else 0.0


def make_sim(rows=2, cols=2, mode="MAX_PRESSURE", reroute_policy="broadcast", controller_params=None,
             green_wave=True, record_positions=False, **router_kw):
    broker = MessageBroker()
    net, agents = build_grid_network(broker, rows, cols, mode=mode, controller_params=controller_params,
                                     green_wave=green_wave)
    router = AStarRouter(net, **router_kw)
    return TrafficSimulator(net, agents, broker, router=router, reroute_policy=reroute_policy,
                            record_positions=record_positions)


def boundary_od_pairs(rows: int, cols: int) -> List[tuple]:
    ends = [f"N{c}" for c in range(cols)] + [f"S{c}" for c in range(cols)] + \
           [f"W{r}" for r in range(rows)] + [f"E{r}" for r in range(rows)]
    return [(a, b) for a in ends for b in ends if a != b and a[0] != b[0]]


# ---------------------------------------------------------------- S1
def scenario_1_fixed_vs_maxpressure(seeds=range(1, 11), rate=0.35, duration=300) -> Dict[str, Any]:
    configs = {
        "Fixed-Time (20s green)": dict(mode="FIXED_TIME"),
        "Max-Pressure, no green-wave": dict(mode="MAX_PRESSURE", green_wave=False),
        "Max-Pressure + green-wave": dict(mode="MAX_PRESSURE", green_wave=True),
    }
    rows = {}
    for name, cfg in configs.items():
        runs = []
        for s in seeds:
            sim = make_sim(**cfg)
            sim.spawn_poisson_demand(OD_2x2, rate, duration, random.Random(s))
            sim.run_until_clear(3000)
            runs.append(sim.summary())
        rows[name] = {k: _mean([r[k] for r in runs]) for k in
                      ("spawned", "arrived", "avg_wait_s", "p95_wait_s", "avg_travel_s", "avg_stops", "fuel_l",
                       "max_queue", "messages")}
        rows[name]["avg_wait_std"] = _std([r["avg_wait_s"] for r in runs])
        rows[name]["per_seed_wait"] = [r["avg_wait_s"] for r in runs]
    fixed = rows["Fixed-Time (20s green)"]["avg_wait_s"]
    full = rows["Max-Pressure, no green-wave"]["avg_wait_s"]
    return {"rows": rows, "seeds": list(seeds), "rate_veh_per_s": rate, "demand_window_s": duration,
            "wait_reduction_pct": round(100 * (fixed - full) / fixed, 1)}


# ---------------------------------------------------------------- S2
def scenario_2_rush_hour(seeds=range(1, 6), ns_rate=0.45, ew_rate=0.07, duration=300) -> Dict[str, Any]:
    ns_od = [("N0", "S0"), ("N1", "S1"), ("S0", "N0"), ("S1", "N1")]
    ew_od = [("W0", "E0"), ("W1", "E1"), ("E0", "W0"), ("E1", "W1")]
    configs = {
        "Fixed-Time": dict(mode="FIXED_TIME"),
        "Max-Pressure WITHOUT anti-starvation": dict(mode="MAX_PRESSURE", controller_params=dict(
            starvation_weight=0.0, max_green_time=10_000.0)),
        "Max-Pressure WITH anti-starvation": dict(mode="MAX_PRESSURE"),
    }
    out = {}
    for name, cfg in configs.items():
        agg = []
        for s in seeds:
            sim = make_sim(**cfg)
            rng = random.Random(s)
            sim.spawn_poisson_demand(ns_od, ns_rate, duration, rng)
            sim.spawn_poisson_demand(ew_od, ew_rate, duration, rng)
            sim.run_until_clear(1500)
            is_ew = lambda v: v.origin_node[0] in "WE"
            is_ns = lambda v: v.origin_node[0] in "NS"
            agg.append({
                "all": sim.summary(),
                "ns": sim.summary(is_ns),
                "ew": sim.summary(is_ew),
            })
        out[name] = {
            "avg_wait_all_s": _mean([a["all"]["avg_wait_s"] for a in agg]),
            "avg_wait_ns_s": _mean([a["ns"]["avg_wait_s"] for a in agg]),
            "avg_wait_ew_s": _mean([a["ew"]["avg_wait_s"] for a in agg]),
            "max_wait_ew_s": max(a["ew"]["max_wait_s"] for a in agg),
            "ew_vehicles_starved": sum(a["ew"]["unfinished"] for a in agg),
            "ew_vehicles_total": sum(a["ew"]["spawned"] for a in agg),
            "arrived_pct": _mean([100 * a["all"]["arrived"] / a["all"]["spawned"] for a in agg]),
        }
    return {"rows": out, "ns_rate": ns_rate, "ew_rate": ew_rate,
            "ns_share_pct": round(100 * ns_rate / (ns_rate + ew_rate), 1)}


# ---------------------------------------------------------------- S3
def scenario_3_emergency(seeds=range(1, 11), bg_rate=0.3) -> Dict[str, Any]:
    bg_od = [("W0", "E0"), ("W1", "E1"), ("E0", "W0"), ("E1", "W1"), ("N1", "S1"), ("S1", "N1")]
    res = {}
    msgs_example = []
    for pre in (False, True):
        ev_t, ev_w, bg_w = [], [], []
        for s in seeds:
            sim = make_sim()
            sim.spawn_poisson_demand(bg_od, bg_rate, 200, random.Random(s))
            ev = sim.spawn_emergency_vehicle("S0", "N0", departure_time=60 + (s % 5) * 3, preemption_enabled=pre)
            sim.run_until_clear(3000)
            ev_t.append(ev.total_travel_time)
            ev_w.append(ev.total_wait_time)
            bg_w.append(sim.summary(lambda v: v is not ev)["avg_wait_s"])
            if pre and not msgs_example:
                msgs_example = [m.short() for m in sim.broker.history()
                                if m.ontology == "emergency-preemption"]
        res["with_preemption" if pre else "without_preemption"] = {
            "ev_travel_s": _mean(ev_t), "ev_travel_std": _std(ev_t), "ev_wait_s": _mean(ev_w),
            "ev_worst_travel_s": max(ev_t), "background_avg_wait_s": _mean(bg_w)}
    a, b = res["without_preemption"], res["with_preemption"]
    res["time_saved_s"] = round(a["ev_travel_s"] - b["ev_travel_s"], 1)
    res["time_saved_pct"] = round(100 * (a["ev_travel_s"] - b["ev_travel_s"]) / a["ev_travel_s"], 1)
    res["background_cost_s"] = round(b["background_avg_wait_s"] - a["background_avg_wait_s"], 2)
    res["protocol_trace"] = msgs_example
    return res


# ---------------------------------------------------------------- S4
def _incident_run(pol, seed, kind, n_target=12, clear_at=None):
    sim = make_sim(reroute_policy=pol)
    sim.spawn_poisson_demand([("N1", "S1"), ("S1", "N1"), ("W1", "E1"), ("E1", "W1")], 0.15, 200,
                             random.Random(seed))
    targets = [sim.spawn_vehicle("W0", "E0", departure_time=15 + i * 4) for i in range(n_target)]
    if kind == "closure":
        sim.schedule(10, lambda sm: sm.coordinator.report_incident("INT_00", "INT_01"))
    else:
        sim.schedule(10, lambda sm: sm.coordinator.report_slowdown("INT_00", "INT_01", speed_mps=2.0))
    if clear_at:
        sim.schedule(clear_at, lambda sm: sm.coordinator.clear_incident("INT_00", "INT_01"))
    sim.run_until_clear(600)
    done = [v for v in targets if v.has_arrived]
    return sim, targets, done


def scenario_4_incident(seeds=range(1, 6), n_target=12) -> Dict[str, Any]:
    """
    Part A - full closure of INT_00->INT_01 : does the system survive at all?
    Part B - slowdown (one lane closed, 2 m/s): does the TMC broadcast help vehicles avoid it EARLY?
    """
    out = {"closure": {}, "slowdown": {}}
    example = {}
    for kind in ("closure", "slowdown"):
        for pol in ("static", "local", "broadcast"):
            rows = []
            for s in seeds:
                sim, targets, done = _incident_run(pol, s, kind, n_target)
                used_bad = sum(1 for v in targets
                               if any(a == "INT_00" and b == "INT_01" for a, b in zip(v.planned_path, v.planned_path[1:])))
                rows.append({"arrived": len(done), "rerouted": sum(v.reroute_count > 0 for v in targets),
                             "avg_travel": _mean([v.total_travel_time for v in done]) if done else None,
                             "used_incident_road": used_bad})
                if kind == "closure" and pol == "broadcast" and not example:
                    example = {"vehicle": targets[0].agent_id, "final_path": targets[0].planned_path,
                               "alert": [m.short() for m in sim.broker.history(ontology="traffic-incident-management")]}
            out[kind][pol] = {
                "arrived_pct": round(100 * _mean([r["arrived"] for r in rows]) / n_target, 1),
                "rerouted": _mean([r["rerouted"] for r in rows]),
                "drove_through_incident_road": _mean([r["used_incident_road"] for r in rows]),
                "avg_travel_s": _mean([r["avg_travel"] for r in rows]),
            }
    return {"rows": out, "incident": "INT_00->INT_01 at t=10 s (W0->E0 commuters depart from t=15 s)",
            "example": example}


# ---------------------------------------------------------------- S5
def scenario_5_bus_priority(seeds=range(1, 9), n_buses=4, bus_pax=40, car_pax=1.2) -> Dict[str, Any]:
    cross_od = [("N0", "S0"), ("S0", "N0"), ("N1", "S1"), ("S1", "N1")]
    out = {}
    for label, rate in (("light cross-traffic (0.2 veh/s)", 0.2), ("heavy cross-traffic (0.5 veh/s)", 0.5)):
        for tsp in (False, True):
            bt, cw, acc, rej, cars_n = [], [], 0, 0, []
            for s in seeds:
                sim = make_sim()
                sim.spawn_poisson_demand(cross_od, rate, 220, random.Random(s))
                buses = [sim.spawn_bus("W0", "E0", departure_time=20 + 45 * i, tsp_enabled=tsp) for i in range(n_buses)]
                sim.run_until_clear(3000)
                bt += [b.total_travel_time for b in buses]
                car = sim.summary(lambda v: v.agent_type.value == "VEHICLE")
                cw.append(car["avg_wait_s"])
                cars_n.append(car["arrived"])
                acc += sum(len(b.accepted) for b in buses)
                rej += sum(len(b.rejected) for b in buses)
            out[f"{label} | TSP {'ON' if tsp else 'OFF'}"] = {
                "bus_travel_s": _mean(bt), "car_avg_wait_s": _mean(cw), "proposals_accepted": acc,
                "proposals_rejected": rej, "cars_per_run": _mean(cars_n), "bus_travel_list": bt}
    # passenger-weighted delay change per run
    for label in ("light cross-traffic (0.2 veh/s)", "heavy cross-traffic (0.5 veh/s)"):
        off, on = out[f"{label} | TSP OFF"], out[f"{label} | TSP ON"]
        bus_gain = (off["bus_travel_s"] - on["bus_travel_s"]) * n_buses * bus_pax
        car_loss = (on["car_avg_wait_s"] - off["car_avg_wait_s"]) * on["cars_per_run"] * car_pax
        on["passenger_seconds_saved_by_bus"] = round(bus_gain, 0)
        on["passenger_seconds_lost_by_cars"] = round(car_loss, 0)
        on["net_passenger_seconds_saved"] = round(bus_gain - car_loss, 0)
    for v in out.values():
        v.pop("bus_travel_list", None)
    return {"rows": out, "bus_passengers": bus_pax, "car_occupancy": car_pax}


# ---------------------------------------------------------------- S6
def _eval_policy(params, seeds, rate=0.4, duration=300):
    waits = []
    for s in seeds:
        sim = make_sim(controller_params=params)
        rng = random.Random(1000 + s)
        # asymmetric, time-varying demand: harder than S1 so tuning has something to find
        sim.spawn_poisson_demand(OD_2x2, rate, duration, rng,
                                 weights=[3, 3, 1, 1, 3, 3, 1, 1, 1, 1, 1, 1])
        sim.run_until_clear(4000)
        waits.append(sim.summary()["avg_wait_s"])
    return sum(waits) / len(waits)


def scenario_6_hill_climbing(train_seeds=(1, 2, 3), val_seeds=(11, 12, 13, 14, 15)) -> Dict[str, Any]:
    from src.agents.coordinator_agent import CentralCoordinatorAgent
    sim = make_sim()
    coord = sim.coordinator
    coord.intersection_ids = list(sim.intersections)
    t0 = time.time()
    res = coord.tune_signal_policy(lambda p: _eval_policy(p, train_seeds), DEFAULT_PARAMS, restarts=2, max_iters=12)
    elapsed = time.time() - t0
    # the PARAMETER_UPDATE messages reach the intersections on the next tick
    sim.step()
    applied = {k: getattr(sim.intersections["INT_00"].controller, k) for k in res.best_params}
    return {
        "default_params": dict(DEFAULT_PARAMS),
        "tuned_params": res.best_params,
        "applied_on_INT_00": applied,
        "train_default_wait_s": round(_eval_policy(DEFAULT_PARAMS, train_seeds), 2),
        "train_tuned_wait_s": round(res.best_cost, 2),
        "val_default_wait_s": round(_eval_policy(DEFAULT_PARAMS, val_seeds), 2),
        "val_tuned_wait_s": round(_eval_policy(res.best_params, val_seeds), 2),
        "val_fixed_time_wait_s": round(_eval_policy_fixed(val_seeds), 2),
        "evaluations": res.evaluations,
        "runtime_s": round(elapsed, 1),
        "trace": res.trace,
    }


def _eval_policy_fixed(seeds, rate=0.4, duration=300):
    waits = []
    for s in seeds:
        sim = make_sim(mode="FIXED_TIME")
        sim.spawn_poisson_demand(OD_2x2, rate, duration, random.Random(1000 + s),
                                 weights=[3, 3, 1, 1, 3, 3, 1, 1, 1, 1, 1, 1])
        sim.run_until_clear(4000)
        waits.append(sim.summary()["avg_wait_s"])
    return sum(waits) / len(waits)


# ---------------------------------------------------------------- S7
def congested_grid(n=6, seed=5, block=True):
    """n x n grid with random queues on the roads (so BPR costs differ) and one blocked road."""
    sim = make_sim(n, n)
    rng = random.Random(seed)
    for seg in sim.network.segments.values():
        k = rng.choice([0, 0, 0, 2, 5, 10, 16, 19])      # vehicles already on the road
        seg.vehicles = [f"dummy_{seg.segment_id}_{i}" for i in range(k)]
    if block:
        mid = n // 2
        sim.network.set_incident(f"INT_{mid}{mid - 1}" if n <= 10 else f"INT_{mid}_{mid-1}",
                                 f"INT_{mid}{mid}" if n <= 10 else f"INT_{mid}_{mid}", True)
    return sim


def scenario_7_search_comparison(n=6, pairs=200, seed=9) -> Dict[str, Any]:
    sim = congested_grid(n)
    net = sim.network
    rng = random.Random(seed)
    ends = [x for x in net.node_positions if not x.startswith("INT")]
    ods = [tuple(rng.sample(ends, 2)) for _ in range(pairs)]
    algos = [("bfs", "euclidean", "BFS"), ("ucs", "euclidean", "UCS (Dijkstra)"),
             ("greedy", "euclidean", "Greedy Best-First"), ("astar", "euclidean", "A* (Euclidean h)"),
             ("astar", "manhattan", "A* (Manhattan h)"), ("weighted_astar", "euclidean", "Weighted A* (w=2)")]
    ref = AStarRouter(net, algorithm="ucs")
    opt = {od: ref.search(*od).cost for od in ods}
    rows = {}
    for algo, h, label in algos:
        r = AStarRouter(net, algorithm=algo, heuristic=h, weight=2.0)
        exp, front, gap, optimal, t0 = [], [], [], 0, time.perf_counter()
        for od in ods:
            res = r.search(*od)
            exp.append(res.nodes_expanded)
            front.append(res.max_frontier)
            g = (res.cost - opt[od]) / opt[od] * 100 if opt[od] > 0 else 0.0
            gap.append(g)
            optimal += g < 1e-6
        dt = (time.perf_counter() - t0) / len(ods) * 1000
        rows[label] = {"avg_nodes_expanded": round(st.mean(exp), 1), "avg_max_frontier": round(st.mean(front), 1),
                       "optimal_pct": round(100 * optimal / len(ods), 1), "avg_extra_cost_pct": round(st.mean(gap), 2),
                       "worst_extra_cost_pct": round(max(gap), 1), "ms_per_search": round(dt, 3)}
    return {"grid": f"{n}x{n} ({len(net.node_positions)} nodes, {len(net.segments)} roads)", "od_pairs": pairs,
            "rows": rows}


def astar_trace(start="W0", goal="E2", n=3):
    """Step-by-step A* trace on a small congested grid (for the demo and slides)."""
    sim = congested_grid(n, seed=3, block=True)
    r = AStarRouter(sim.network, algorithm="astar")
    res = r.search(start, goal, record_trace=True)
    ucs = r.search(start, goal, algorithm="ucs")
    blocked = [s.segment_id for s in sim.network.segments.values() if s.is_blocked]
    return {"result": res, "ucs": ucs, "blocked": blocked, "v_max": r.global_v_max}


# ---------------------------------------------------------------- S8
def scenario_8_scalability(sizes=(2, 3, 4, 5, 6, 8), duration=300, seed=4) -> Dict[str, Any]:
    rows = {}
    for n in sizes:
        rate = 0.09 * 4 * n / 2          # demand grows with the number of entry roads
        res = {}
        for mode in ("FIXED_TIME", "MAX_PRESSURE"):
            sim = make_sim(n, n, mode=mode)
            sim.spawn_poisson_demand(boundary_od_pairs(n, n), rate, duration, random.Random(seed))
            t0 = time.perf_counter()
            sim.run_until_clear(5000)
            wall = time.perf_counter() - t0
            s = sim.summary()
            res[mode] = s
            res[mode + "_ms_per_tick"] = round(1000 * wall / sim.sim_time, 2)
            res[mode + "_agents"] = len(sim.intersections) + len(sim.vehicles) + 1
        rows[f"{n}x{n}"] = {
            "intersections": n * n,
            "vehicles": res["MAX_PRESSURE"]["spawned"],
            "total_agents": res["MAX_PRESSURE_agents"],
            "fixed_avg_wait_s": res["FIXED_TIME"]["avg_wait_s"],
            "mp_avg_wait_s": res["MAX_PRESSURE"]["avg_wait_s"],
            "wait_reduction_pct": round(100 * (res["FIXED_TIME"]["avg_wait_s"] - res["MAX_PRESSURE"]["avg_wait_s"])
                                        / res["FIXED_TIME"]["avg_wait_s"], 1),
            "messages": res["MAX_PRESSURE"]["messages"],
            "ms_per_tick": res["MAX_PRESSURE_ms_per_tick"],
            "arrived_pct": round(100 * res["MAX_PRESSURE"]["arrived"] / res["MAX_PRESSURE"]["spawned"], 1),
        }
    return {"rows": rows}


def run_all(fast: bool = False) -> Dict[str, Any]:
    k = 3 if fast else None
    return {
        "S1": scenario_1_fixed_vs_maxpressure(seeds=range(1, (k or 10) + 1)),
        "S2": scenario_2_rush_hour(seeds=range(1, (k or 5) + 1)),
        "S3": scenario_3_emergency(seeds=range(1, (k or 10) + 1)),
        "S4": scenario_4_incident(seeds=range(1, (k or 5) + 1)),
        "S5": scenario_5_bus_priority(seeds=range(1, (k or 8) + 1)),
        "S6": scenario_6_hill_climbing(),
        "S7": scenario_7_search_comparison(),
        "S8": scenario_8_scalability(sizes=(2, 3, 4) if fast else (2, 3, 4, 5, 6, 8)),
    }
