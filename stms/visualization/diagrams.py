"""Two report diagrams: the agent architecture and the message sequence of the demo (no bus)."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

PURPLE, BLUE, GREEN, ORANGE, GREY = "#4b3aa7", "#2a78d6", "#1baf7a", "#eb6834", "#9a9893"


def _box(ax, x, y, w, h, title, body, col):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.0,rounding_size=0.12", fc="white", ec=col, lw=2.5))
    ax.add_patch(FancyBboxPatch((x, y + h - 0.42), w, 0.42, boxstyle="round,pad=0.0,rounding_size=0.12", fc=col, ec=col))
    ax.text(x + w / 2, y + h - 0.21, title, color="white", ha="center", va="center", fontsize=10, fontweight="bold")
    ax.text(x + w / 2, y + (h - 0.42) / 2, body, ha="center", va="center", fontsize=8.6, linespacing=1.5)


def _arrow(ax, p, q, col, text=None, tx=None, ty=None, both=False):
    ax.annotate("", xy=q, xytext=p, arrowprops=dict(arrowstyle="<->" if both else "->", color=col, lw=2))
    if text:
        ax.text(tx if tx is not None else (p[0] + q[0]) / 2, ty if ty is not None else (p[1] + q[1]) / 2, text,
                color=col, ha="center", va="center", fontsize=7.8, linespacing=1.3,
                bbox=dict(fc="white", ec="none", pad=1.5))


def architecture(path):
    fig, ax = plt.subplots(figsize=(8.4, 5.2)); ax.set_xlim(0, 12); ax.set_ylim(0, 7.4); ax.axis("off")
    ax.text(6, 7.2, "Every arrow between agents is a FIPA-ACL message through the broker, never a direct function call",
            ha="center", fontsize=8.5, style="italic", color="#555")
    _box(ax, 4.2, 5.1, 3.6, 1.8, "Coordinator (TMC) - M5", "telemetry: congestion index\nincident alerts  |  hill-climbing tuner", PURPLE)
    _box(ax, 0.2, 2.5, 3.2, 1.9, "Ambulance agent - M4", "asks junctions for a green corridor\n(PREEMPT_OVERRIDE), then CANCEL\n(extends the Vehicle agent)", ORANGE)
    _box(ax, 4.4, 2.5, 3.2, 1.9, "Intersection agents - M2", "Max-Pressure + anti-starvation\nFixed-time baseline\ngreen-wave INFORM to neighbours", BLUE)
    _box(ax, 8.8, 2.5, 3.0, 1.9, "Vehicle agents - M3", "A* on own beliefs\nre-plan on alerts\ncar-following, headway", GREEN)
    _box(ax, 0.2, 0.2, 11.6, 1.3, "Environment & simulator - M1", "road graph G = (V, E)  |  BPR travel time  |  saturation flow 1800 veh/h  |  1 s ticks  |  Poisson demand", GREY)
    _arrow(ax, (4.2, 6.0), (1.8, 4.4), PURPLE, "BROADCAST (same alerts)", 2.2, 5.3)
    _arrow(ax, (6.0, 5.1), (6.0, 4.4), PURPLE, "INFORM\nPARAMETER_UPDATE", 6.0, 4.75)
    _arrow(ax, (7.8, 6.0), (10.3, 4.4), PURPLE, "BROADCAST INCIDENT_ALERT\nINFORM CONGESTION_WARNING", 9.9, 5.4)
    _arrow(ax, (3.4, 3.7), (4.4, 3.7), ORANGE, "PREEMPT_\nOVERRIDE", 3.9, 4.15)
    _arrow(ax, (4.4, 3.1), (3.4, 3.1), BLUE, "AGREE", 3.9, 2.85)
    for x, t in ((1.8, "sense / move"), (6.0, "sense queues / set lights"), (10.3, "sense signal / move")):
        _arrow(ax, (x, 2.5), (x, 1.5), GREY, both=True); ax.text(x + 0.15, 2.0, t, fontsize=7.8, color="#555", va="center")
    fig.savefig(path, dpi=170, bbox_inches="tight"); plt.close(fig)


def sequence(path):
    lanes = ["Traffic centre", "Car", "Ambulance", "Junction J20", "Junction J10", "Junction J00"]
    cols = [PURPLE, GREEN, "#dc2626", BLUE, BLUE, BLUE]
    xs = [1, 2.9, 4.8, 6.7, 8.6, 10.5]
    R = "#dc2626"
    # real messages of the demo run (python run_simulation.py --protocol shows the same log)
    msgs = [(10, 0, 1, "BROADCAST alert", PURPLE),
            (10, 1, 1, "Car: closed road on my route -> A* again", GREEN),
            (14, 2, 3, "PREEMPT_OVERRIDE (NS, ETA 12 s)", R),
            (15, 3, 2, "AGREE", BLUE),
            (25, 2, 4, "PREEMPT_OVERRIDE (NS, ETA 14.9 s)", R),
            (26, 4, 2, "AGREE", BLUE),
            (27, 2, 3, "CANCEL (crossed J20)", R),
            (37, 2, 5, "PREEMPT_OVERRIDE (NS, ETA 14.9 s)", R),
            (38, 5, 2, "AGREE", BLUE),
            (39, 2, 4, "CANCEL (crossed J10)", R)]
    fig, ax = plt.subplots(figsize=(8.4, 4.8)); ax.set_xlim(-0.6, 11.4); ax.set_ylim(len(msgs) + 0.8, -1.2); ax.axis("off")
    for x, l, c in zip(xs, lanes, cols):
        ax.text(x, -0.7, l, ha="center", va="center", color="white", fontsize=8.5, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.35", fc=c, ec=c)); ax.plot([x, x], [-0.3, len(msgs) + 0.5], color="#bbb", lw=1, zorder=0)
    for i, (t, a, b, text, col) in enumerate(msgs):
        y = i + 0.4
        ax.text(-0.55, y, f"t={t:g}s", fontsize=7.5, color="#666", va="center", ha="left")
        if a == b:
            ax.text(xs[a] + 0.1, y, text, fontsize=7.8, color=col, va="center", ha="left"); ax.plot(xs[a], y, "o", color=col, ms=4)
        else:
            ax.annotate("", xy=(xs[b], y), xytext=(xs[a], y), arrowprops=dict(arrowstyle="->", color=col, lw=1.6))
            ax.text((xs[a] + xs[b]) / 2, y - 0.22, text, fontsize=7.5, color=col, ha="center", va="center")
    fig.savefig(path, dpi=170, bbox_inches="tight"); plt.close(fig)


def make_diagrams(outdir):
    a, s = os.path.join(outdir, "architecture.png"), os.path.join(outdir, "sequence.png")
    architecture(a); sequence(s)
    return [a, s]
