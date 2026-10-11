"""
Smart Traffic Management System (STMS) - demo runner for the Review 2 presentation.

    python run_simulation.py --all              every scenario, one after another (~1 min)
    python run_simulation.py --scenario 3       one scenario: 1, 2, 3, 4, 5, 6, 7, 8
    python run_simulation.py --astar-trace      step-by-step A* expansion table
    python run_simulation.py --protocol         live FIPA-ACL message log (ambulance + bus + incident)
"""

import argparse
import io
import os
import random
import sys
import time

if sys.platform.startswith("win"):
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from rich.console import Console
from rich.panel import Panel
from rich.table import Table

from src import scenarios as S

console = Console()


def banner():
    console.print(Panel.fit(
        "[bold cyan]SMART TRAFFIC MANAGEMENT SYSTEM[/]  -  multi-agent AI case study\n"
        "[white]Agents:[/] Intersection (Max-Pressure) | Vehicle (A*) | Ambulance (preemption) | "
        "Bus (negotiated priority) | Coordinator/TMC (hill climbing)\n"
        "[dim]Team of 5 - see README.md for who owns which module[/]", border_style="cyan"))


def table(title, cols, rows, style="cyan"):
    t = Table(title=title, border_style=style, title_style="bold")
    for i, c in enumerate(cols):
        t.add_column(c, justify="left" if i == 0 else "center")
    for r in rows:
        t.add_row(*[str(x) for x in r])
    console.print(t)


def show_s1():
    console.rule("[bold]S1  Fixed-Time vs Max-Pressure (2x2 grid, Poisson demand, 10 seeds)")
    r = S.scenario_1_fixed_vs_maxpressure()
    table("Averaged over 10 random seeds - every vehicle driven to its destination",
          ["Controller", "Avg wait (s)", "95th pct wait", "Avg travel (s)", "Stops/veh", "Fuel (L)", "Messages"],
          [[k, f"{v['avg_wait_s']} +/- {v['avg_wait_std']}", v["p95_wait_s"], v["avg_travel_s"], v["avg_stops"],
            v["fuel_l"], v["messages"]] for k, v in r["rows"].items()])
    console.print(f"[green]Max-Pressure cuts average wait by {r['wait_reduction_pct']}% vs Fixed-Time.[/]")
    console.print("[dim]Green-wave hand-off does not help on random grid demand.[/]")


def show_s2():
    console.rule("[bold]S2  Rush hour (heavy N-S) and the anti-starvation term")
    r = S.scenario_2_rush_hour()
    table(f"{r['ns_share_pct']}% of traffic on the N-S axis",
          ["Controller", "Wait N-S (s)", "Wait E-W (s)", "Worst E-W wait (s)", "E-W cars never served"],
          [[k, v["avg_wait_ns_s"], v["avg_wait_ew_s"], v["max_wait_ew_s"],
            f"{v['ew_vehicles_starved']} / {v['ew_vehicles_total']}"] for k, v in r["rows"].items()])
    console.print("[green]Without the starvation term pure Max-Pressure leaves minority cars stuck forever.[/]")


def show_s3():
    console.rule("[bold]S3  Ambulance green corridor (same Max-Pressure controller, preemption off vs on)")
    r = S.scenario_3_emergency()
    a, b = r["without_preemption"], r["with_preemption"]
    table("10 seeds with background cross traffic",
          ["Mode", "Ambulance travel (s)", "Ambulance wait (s)", "Worst case (s)", "Other cars' avg wait (s)"],
          [["No preemption", a["ev_travel_s"], a["ev_wait_s"], a["ev_worst_travel_s"], a["background_avg_wait_s"]],
           ["PREEMPT_OVERRIDE", b["ev_travel_s"], b["ev_wait_s"], b["ev_worst_travel_s"], b["background_avg_wait_s"]]],
          style="red")
    console.print(f"[green]{r['time_saved_s']} s faster ({r['time_saved_pct']}%), cost to other traffic "
                  f"{r['background_cost_s']:+} s.[/]")
    for line in r["protocol_trace"]:
        console.print("  [dim]" + line + "[/]")


def show_s4():
    console.rule("[bold]S4  Accident on INT_00->INT_01: static vs local sensing vs TMC broadcast")
    r = S.scenario_4_incident()
    for kind, label in (("closure", "Road CLOSED"), ("slowdown", "Road SLOWED to 2 m/s (one lane closed)")):
        table(label, ["Re-routing policy", "Arrived", "Re-routed", "Drove through incident", "Avg travel (s)"],
              [[p, f"{v['arrived_pct']}%", v["rerouted"], v["drove_through_incident_road"], v["avg_travel_s"]]
               for p, v in r["rows"][kind].items()], style="magenta")
    console.print("[green]Only the broadcast lets vehicles avoid a SLOW road before they are stuck on it.[/]")


def show_s5():
    console.rule("[bold]S5  Bus Transit Signal Priority - negotiation, not override")
    r = S.scenario_5_bus_priority()
    table("4 buses x 40 passengers crossing N-S traffic",
          ["Case", "Bus travel (s)", "Car wait (s)", "Accepted", "Rejected", "Net passenger-s saved"],
          [[k, v["bus_travel_s"], v["car_avg_wait_s"], v["proposals_accepted"], v["proposals_rejected"],
            v.get("net_passenger_seconds_saved", "-")] for k, v in r["rows"].items()], style="yellow")
    console.print("[yellow]Honest trade-off: cars wait longer, but more people get through sooner.[/]")


def show_s6():
    console.rule("[bold]S6  Coordinator tunes signal parameters with Hill Climbing")
    with console.status("hill climbing (about 15 s)..."):
        r = S.scenario_6_hill_climbing()
    table("Search trace", ["Restart", "Iter", "Parameters", "Avg wait (s)", "Move"],
          [[t["restart"], t["iter"], t["params"], t["cost"], t["move"]] for t in r["trace"]])
    table("Result on UNSEEN validation traffic", ["Policy", "Avg wait (s)"],
          [["Fixed-Time", r["val_fixed_time_wait_s"]], ["Max-Pressure default", r["val_default_wait_s"]],
           ["Max-Pressure tuned", r["val_tuned_wait_s"]]])
    console.print(f"Tuned parameters pushed to all intersections via INFORM PARAMETER_UPDATE: {r['applied_on_INT_00']}")


def show_s7():
    console.rule("[bold]S7  Route search algorithms (vehicle agent)")
    r = S.scenario_7_search_comparison()
    table(f"{r['grid']}, {r['od_pairs']} random trips, BPR congested costs, one closed road",
          ["Algorithm", "Nodes expanded", "Max frontier", "Optimal routes", "Avg extra cost", "ms / search"],
          [[k, v["avg_nodes_expanded"], v["avg_max_frontier"], f"{v['optimal_pct']}%", f"{v['avg_extra_cost_pct']}%",
            v["ms_per_search"]] for k, v in r["rows"].items()])


def show_s8():
    console.rule("[bold]S8  Scalability")
    with console.status("running 2x2 ... 8x8 grids"):
        r = S.scenario_8_scalability()
    table("Decentralised control: each intersection decides with local data only",
          ["Grid", "Intersections", "Vehicles", "Fixed wait (s)", "Max-P wait (s)", "Reduction", "Messages",
           "ms / tick"],
          [[k, v["intersections"], v["vehicles"], v["fixed_avg_wait_s"], v["mp_avg_wait_s"],
            f"{v['wait_reduction_pct']}%", v["messages"], v["ms_per_tick"]] for k, v in r["rows"].items()])


def show_astar_trace():
    console.rule("[bold]A* step-by-step (f = g + h)")
    from generate_results import astar_trace_text
    console.print(astar_trace_text())


def show_protocol():
    console.rule("[bold]Live FIPA-ACL message log")
    from generate_results import demo_sim
    sim = demo_sim()
    seen = 0
    for _ in range(160):
        sim.step()
        hist = sim.broker.history()
        for m in hist[seen:]:
            if m.ontology != "green-wave":
                colour = {"PREEMPT_OVERRIDE": "red", "AGREE": "green", "PROPOSE": "yellow",
                          "ACCEPT_PROPOSAL": "green", "REJECT_PROPOSAL": "red", "BROADCAST": "magenta",
                          "CANCEL": "cyan"}.get(m.performative.value, "white")
                console.print(f"[{colour}]{m.short()}[/]")
        seen = len(hist)
    console.print(f"\nTotal messages: {sim.broker.total_messages_dispatched}  by type: {dict(sim.broker.counts)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--scenario", choices=["1", "2", "3", "4", "5", "6", "7", "8"])
    ap.add_argument("--astar-trace", action="store_true")
    ap.add_argument("--protocol", action="store_true")
    a = ap.parse_args()
    banner()
    shows = {"1": show_s1, "2": show_s2, "3": show_s3, "4": show_s4, "5": show_s5,
             "6": show_s6, "7": show_s7, "8": show_s8}
    if a.scenario:
        shows[a.scenario]()
    elif a.astar_trace:
        show_astar_trace()
    elif a.protocol:
        show_protocol()
    else:
        for f in shows.values():
            f()
        show_astar_trace()


if __name__ == "__main__":
    main()
