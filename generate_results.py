"""
Runs every evaluation scenario and regenerates all evidence used in the report and slides:

    results/results.json           all numbers (single source of truth)
    results/figures/*.png          charts
    demo.html                      the one-page demo (re-routing)
    results/astar_trace.txt        step-by-step A* trace
    results/protocol_trace.txt     FIPA-ACL message log of the demo run

Usage:   python generate_results.py            (full, ~1 minute)
         python generate_results.py --fast     (fewer seeds, for a quick check)
"""

import argparse
import json
import os
import random
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from src import scenarios as S
from src.visualization.charts import make_all_charts
from src.visualization.diagrams import make_diagrams

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")


def demo_sim():
    sim = S.make_sim()
    sim.spawn_poisson_demand(S.OD_2x2, 0.4, 200, random.Random(3))
    sim.spawn_emergency_vehicle("S0", "N0", departure_time=62)
    sim.schedule(30, lambda s: s.coordinator.report_slowdown("INT_10", "INT_11", 2.0))
    return sim


def astar_trace_text() -> str:
    t = S.astar_trace()
    r, u = t["result"], t["ucs"]
    lines = [f"A* trace on a congested 3x3 grid, W0 -> E2   (v_max = {t['v_max']:.2f} m/s, h = Euclidean/v_max)",
             f"Blocked road(s): {', '.join(t['blocked'])}", "",
             f"{'step':>4}  {'expand':<8} {'g(n)':>7} {'h(n)':>7} {'f(n)':>7}   frontier (f, node) after pop"]
    for row in r.trace:
        fr = ", ".join(f"({f},{n})" for f, n in row["frontier"])
        lines.append(f"{row['step']:>4}  {row['expand']:<8} {row['g']:>7} {row['h']:>7} {row['f']:>7}   {fr}")
    lines += ["", f"A*  path: {' -> '.join(r.path)}", f"    cost = {r.cost:.2f} s, nodes expanded = {r.nodes_expanded}",
              f"UCS path: {' -> '.join(u.path)}", f"    cost = {u.cost:.2f} s, nodes expanded = {u.nodes_expanded}",
              "Same optimal cost; A* expands fewer nodes because h(n) steers it toward the goal."]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fast", action="store_true")
    args = ap.parse_args()
    os.makedirs(os.path.join(OUT, "figures"), exist_ok=True)

    t0 = time.time()
    print("Running scenarios ...")
    R = S.run_all(fast=args.fast)

    # queue time-series for one seed (Fixed vs Max-Pressure)
    R["queue_series"] = {}
    for mode in ("FIXED_TIME", "MAX_PRESSURE"):
        sim = S.make_sim(mode=mode)
        sim.spawn_poisson_demand(S.OD_2x2, 0.35, 300, random.Random(1))
        sim.run_until_clear(3000)
        R["queue_series"][mode] = sim.metrics.queue_series

    # protocol trace from the demo run
    sim = demo_sim()
    sim.run(160)
    proto = [m.short() for m in sim.broker.history() if m.ontology != "green-wave"][:60]
    with open(os.path.join(OUT, "protocol_trace.txt"), "w", encoding="utf-8") as f:
        f.write("FIPA-ACL messages exchanged in the demo run (green-wave INFORMs omitted)\n\n" + "\n".join(proto))
    R["demo_protocol_trace"] = proto

    trace = astar_trace_text()
    with open(os.path.join(OUT, "astar_trace.txt"), "w", encoding="utf-8") as f:
        f.write(trace)

    with open(os.path.join(OUT, "results.json"), "w", encoding="utf-8") as f:
        json.dump(R, f, indent=1, default=str)
    print("Making charts ...")
    for p in make_all_charts(R, os.path.join(OUT, "figures")):
        print("  ", os.path.relpath(p))
    make_diagrams(os.path.join(OUT, "figures"))
    import run_demo
    run_demo.build()
    print("demo.html rebuilt")
    print(f"Done in {time.time() - t0:.0f} s. Results in {OUT}")


if __name__ == "__main__":
    main()
