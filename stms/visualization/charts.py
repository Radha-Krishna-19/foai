"""
Report / PPT charts generated from results/results.json.

Owner: Member 1 (Environment & Simulation) - every member supplies their scenario's numbers.
Palette (validated for colour-vision deficiency): blue = proposed MAS, orange = baseline,
aqua = third series. Every bar carries a direct value label, so colour is never the only cue.
"""

import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BLUE, ORANGE, AQUA, GREY = "#2a78d6", "#eb6834", "#1baf7a", "#9a9893"
INK, INK2 = "#0b0b0b", "#52514e"


def _style(ax, title, ylabel=None, xlabel=None):
    ax.set_title(title, fontsize=11, color=INK, loc="left", pad=10)
    if ylabel:
        ax.set_ylabel(ylabel, color=INK2, fontsize=9)
    if xlabel:
        ax.set_xlabel(xlabel, color=INK2, fontsize=9)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color("#cfcdc8")
    ax.tick_params(colors=INK2, labelsize=8)
    ax.yaxis.grid(True, color="#ecebe8", lw=0.8)
    ax.set_axisbelow(True)


def _bars(ax, labels, values, colors, fmt="{:.1f}", width=0.55, err=None):
    xs = range(len(labels))
    bars = ax.bar(xs, values, width=width, color=colors, edgecolor="white", linewidth=2, yerr=err,
                  error_kw=dict(ecolor=INK2, lw=1, capsize=3))
    top = max(v for v in values if v is not None) if values else 1
    for i, (b, v) in enumerate(zip(bars, values)):
        e = err[i] if err else 0.0
        ax.text(b.get_x() + b.get_width() / 2, b.get_height() + e + top * 0.02, fmt.format(v),
                ha="center", va="bottom", fontsize=9, color=INK)
    ax.set_xticks(list(xs))
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylim(0, top * 1.22)


def _grouped(ax, groups, series, colors, fmt="{:.1f}"):
    n = len(series)
    w = 0.8 / n
    top = max(v for vals in series.values() for v in vals) or 1
    for i, (name, vals) in enumerate(series.items()):
        xs = [g + (i - (n - 1) / 2) * w for g in range(len(groups))]
        bars = ax.bar(xs, vals, width=w * 0.92, color=colors[i], label=name, edgecolor="white", linewidth=2)
        for b, v in zip(bars, vals):
            ax.text(b.get_x() + b.get_width() / 2, b.get_height() + top * 0.02, fmt.format(v),
                    ha="center", va="bottom", fontsize=8, color=INK)
    ax.set_xticks(range(len(groups)))
    ax.set_xticklabels(groups, fontsize=8)
    ax.set_ylim(0, top * 1.25)
    ax.legend(frameon=False, fontsize=8)


def _save(fig, outdir, name):
    path = os.path.join(outdir, name)
    fig.tight_layout()
    fig.savefig(path, dpi=180, facecolor="white")
    plt.close(fig)
    return path


def make_all_charts(R: dict, outdir: str) -> list:
    os.makedirs(outdir, exist_ok=True)
    out = []

    # S1
    rows = R["S1"]["rows"]
    names = list(rows)
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    _bars(ax, ["Fixed-Time", "Max-Pressure", "Max-Pressure\n+ green-wave"], [rows[n]["avg_wait_s"] for n in names],
          [ORANGE, BLUE, AQUA], err=[rows[n]["avg_wait_std"] for n in names])
    _style(ax, f"S1  Avg wait per vehicle, 2x2 grid, 10 seeds (-{R['S1']['wait_reduction_pct']}%)",
           "seconds")
    ax.title.set_fontsize(10)
    out.append(_save(fig, outdir, "s1_fixed_vs_maxpressure.png"))


    # S2
    r = R["S2"]["rows"]
    keys = list(r)
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    _grouped(ax, ["Fixed-Time", "Max-Pressure\nno anti-starvation", "Max-Pressure\nwith anti-starvation"],
             {"Heavy N-S approach": [r[k]["avg_wait_ns_s"] for k in keys],
              "Light E-W approach": [r[k]["avg_wait_ew_s"] for k in keys]}, [BLUE, ORANGE])
    _style(ax, "S2  Rush hour (87% N-S): average wait by approach", "seconds")
    k0 = keys[1]
    ax.text(1.2, r[k0]["avg_wait_ew_s"] * 1.09, f"{r[k0]['ew_vehicles_starved']} of {r[k0]['ew_vehicles_total']} E-W cars\nnever served",
            fontsize=8, color=INK2, ha="center", va="bottom")
    ax.set_ylim(0, r[k0]["avg_wait_ew_s"] * 1.45)
    out.append(_save(fig, outdir, "s2_rush_hour_starvation.png"))

    # S3
    r = R["S3"]
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    _grouped(ax, ["Ambulance travel time", "Ambulance wait at red"],
             {"No preemption": [r["without_preemption"]["ev_travel_s"], r["without_preemption"]["ev_wait_s"]],
              "Preemption (MAS)": [r["with_preemption"]["ev_travel_s"], r["with_preemption"]["ev_wait_s"]]},
             [ORANGE, BLUE])
    _style(ax, f"S3  Emergency green corridor: {r['time_saved_pct']}% faster, same Max-Pressure controller", "seconds")
    out.append(_save(fig, outdir, "s3_emergency.png"))

    # S4
    r = R["S4"]["rows"]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.6, 3.4))
    pols = ["static", "local", "broadcast"]
    _bars(a1, ["Static\nroute", "Local\nsensing", "TMC\nbroadcast"], [r["closure"][p]["arrived_pct"] for p in pols],
          [GREY, ORANGE, BLUE], fmt="{:.0f}%")
    _style(a1, "Road CLOSED: vehicles that arrive", "% of 12")
    _bars(a2, ["Static\nroute", "Local\nsensing", "TMC\nbroadcast"], [r["slowdown"][p]["avg_travel_s"] for p in pols],
          [GREY, ORANGE, BLUE])
    _style(a2, "Road SLOWED to 2 m/s: travel time", "seconds")
    fig.suptitle("S4  Incident on INT_00->INT_01", x=0.02, ha="left", fontsize=11)
    out.append(_save(fig, outdir, "s4_incident.png"))

    # S5
    r = R["S5"]["rows"]; c, o = r["Conventional"], r["Ours (STMS)"]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.6, 3.4))
    _bars(a1, ["Conventional", "Ours"], [c["ambulance_trip_s"], o["ambulance_trip_s"]], [GREY, BLUE])
    _style(a1, f"Ambulance trip time: {R['S5']['ambulance_trip_saved_pct']}% faster", "seconds")
    _bars(a2, ["Conventional", "Ours"], [c["avg_wait_s"], o["avg_wait_s"]], [GREY, BLUE])
    _style(a2, f"Average wait per car: {R['S5']['avg_wait_saved_pct']}% less", "seconds")
    fig.suptitle("S5  Conventional city vs our system (same traffic, same accident)", x=0.02, ha="left", fontsize=11)
    out.append(_save(fig, outdir, "s5_conventional_vs_ours.png"))

    # S6
    tr = R["S6"]["trace"]
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    cols = [BLUE, ORANGE, AQUA]
    for rs in sorted({t["restart"] for t in tr}):
        pts = [(t["iter"], t["cost"]) for t in tr if t["restart"] == rs and t["move"] != "local optimum"]
        ax.plot(*zip(*pts), marker="o", ms=5, lw=2, color=cols[rs % 3], label=f"restart {rs}")
        ax.text(pts[-1][0] + 0.1, pts[-1][1], f"{pts[-1][1]:.2f}", fontsize=8, color=INK, va="center")
    ax.legend(frameon=False, fontsize=8)
    _style(ax, "S6  Hill climbing: average wait of the policy at each step", "avg wait (s)", "iteration")
    out.append(_save(fig, outdir, "s6_hill_climbing.png"))

    # S7
    r = R["S7"]["rows"]
    names = list(r)
    short = ["BFS", "UCS", "Greedy", "A*\nEuclid", "A*\nManhattan", "W-A*\nw=2"]
    colors = [GREY, ORANGE, GREY, BLUE, BLUE, AQUA]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(8.4, 3.4))
    _bars(a1, short, [r[n]["avg_nodes_expanded"] for n in names], colors)
    _style(a1, "Nodes expanded (lower = faster)")
    _bars(a2, short, [r[n]["optimal_pct"] for n in names], colors, fmt="{:.0f}%")
    _style(a2, "Routes that are optimal")
    fig.suptitle(f"S7  Route search on a {R['S7']['grid']}, {R['S7']['od_pairs']} random trips",
                 x=0.02, ha="left", fontsize=11)
    out.append(_save(fig, outdir, "s7_search_comparison.png"))

    # S8
    r = R["S8"]["rows"]
    sizes = list(r)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(8.4, 3.4))
    n_int = [r[s]["intersections"] for s in sizes]
    a1.plot(n_int, [r[s]["ms_per_tick"] for s in sizes], marker="o", ms=6, lw=2, color=BLUE)
    for x, s in zip(n_int, sizes):
        a1.text(x, r[s]["ms_per_tick"] * 1.06, s, fontsize=7, ha="center", color=INK2)
    _style(a1, "Compute per simulated second", "ms / tick", "intersections")
    _grouped(a2, sizes, {"Fixed-Time": [r[s]["fixed_avg_wait_s"] for s in sizes],
                         "Max-Pressure": [r[s]["mp_avg_wait_s"] for s in sizes]}, [ORANGE, BLUE], fmt="{:.0f}")
    _style(a2, "Average wait (s)")
    fig.suptitle("S8  Scalability: 2x2 to 8x8 grids", x=0.02, ha="left", fontsize=11)
    out.append(_save(fig, outdir, "s8_scalability.png"))

    # queue time-series
    if "queue_series" in R:
        q = R["queue_series"]
        fig, ax = plt.subplots(figsize=(6.4, 3.2))
        ax.plot(q["FIXED_TIME"], color=ORANGE, lw=2, label="Fixed-Time")
        ax.plot(q["MAX_PRESSURE"], color=BLUE, lw=2, label="Max-Pressure")
        ax.legend(frameon=False, fontsize=8)
        _style(ax, "Vehicles on the network over time (same demand, seed 1)", "vehicles", "time (s)")
        out.append(_save(fig, outdir, "queue_timeseries.png"))
    return out
