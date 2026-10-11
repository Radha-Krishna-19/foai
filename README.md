# Smart Traffic Management System (STMS)
### A multi-agent AI case study: 5 agent types, 5 team members, 43 automated tests

**Problem.** Fixed-timing signals ignore real queues, and drivers on fixed routes keep driving into accidents. **Solution.** Make junctions and cars agents: junctions pick green by queue pressure (Max-Pressure), cars re-plan with A* when told about an accident, the ambulance gets a green corridor. **Result.** Average wait -47% vs fixed timing; with a closed road 100% of cars arrive when re-routing vs 0% with fixed routes.

Intersections, cars, an ambulance and a Traffic Management Centre are **independent agents**.
Each one senses, reasons and acts on its own, and they coordinate only through FIPA-ACL messages.

| Agent | Algorithm it runs | Talks to others by |
|---|---|---|
| Intersection | **Max-Pressure** signal control + anti-starvation | AGREE / ACCEPT / REJECT, green-wave INFORM |
| Vehicle | **A\*** route search (admissible, consistent heuristic) | receives INCIDENT_ALERT, CONGESTION_WARNING |
| Ambulance | look-ahead **preemption** | PREEMPT_OVERRIDE, CANCEL |
| Coordinator (TMC) | **Hill climbing** with random restarts (tunes signal parameters) | BROADCAST, INFORM PARAMETER_UPDATE |

---

## Team and ownership

| Member | Name / Roll No. | Module (code) | Scenario / test ownership |
|---|---|---|---|
| M1 | P.M. Radha Krishna (CB.SC.U4CSE23134) | Environment & simulation: `src/environment/`, `src/demo.py` | S8 scalability, demo page, UT-09..12 |
| M2 | *Member 2* | Intersection agent: `intersection_agent.py`, `max_pressure.py` | S1, S2, UT-13..18 |
| M3 | *Member 3* | Vehicle agent & search: `vehicle_agent.py`, `search_astar.py` | S4, S7, A\* trace, DM-01..03, UT-01..08, 22, 23 |
| M4 | *Member 4* | Ambulance agent: `emergency_agent.py` | S3, S5, IT-01, IT-02 |
| M5 | *Member 5* | Coordinator & protocol: `coordinator_agent.py`, `message.py`, `hill_climbing.py` | S6, IT-05..09, test runner |

---

## Run it

```
pip install -r requirements.txt
START_DEMO.bat                         # Windows: double-click. Same as: python run_demo.py (click roads to add/remove accidents)
python run_demo.py                     # THE DEMO: local page; click roads to add/remove several accidents
python run_demo.py --static            # only write demo.html (works offline by double-click; one accident at a time)
python run_simulation.py --all         # every experiment as a table (about 1 minute)
python run_simulation.py --scenario 3  # one experiment: 1, 2, 3, 4, 5, 6, 7, 8
python run_simulation.py --protocol    # the FIPA-ACL message log
python run_simulation.py --astar-trace # A* expansion table
python tests/run_tests.py              # 43 tests -> results/test_report.json
python generate_results.py             # re-create every number, chart and demo.html
```

### What the demo page shows (and nothing more)

The page runs the same city twice, conventional (fixed-time lights, no alerts, no ambulance priority) and ours, with the same traffic and accident, and compares them. Click a road on the right map to add an accident, click again to remove it; the real simulation re-runs.
Team work split: see `TEAM_SPLIT.md`.

A 3x3 city. Six cars each start on their cheapest route (A*). At t = 10 s an accident closes one road
and the traffic centre broadcasts it. Click a car to see its **original route (dashed)** and, if it had to
change, its **new route (solid)** with a one-line reason. Cars that did not use the closed road keep going.
Traffic lights are Max-Pressure agents; cars stop behind red lights.

---

## Headline results (from `results/results.json`)

| # | Scenario | Result |
|---|---|---|
| S1 | Fixed-Time vs Max-Pressure, 2x2 grid, 10 seeds | avg wait **15.45 s -> 8.11 s (-47.5%)** |
| S2 | Rush hour, 87% N-S | without anti-starvation **14 / 108** cross-street cars never served; with it **0** |
| S3 | Ambulance, preemption off vs on (same controller) | **54.0 s -> 38.0 s (-29.6%)**, 0 s at red lights, other traffic unaffected |
| S4 | Accident on INT_00->INT_01 | closure: static routing 0% arrive, re-routing 100%; slowdown: broadcast **144.9 s -> 95.6 s** |
| S5 | Conventional city vs our system (same traffic, accident J01-J11, 1 ambulance) | ambulance **126 s -> 73 s (-42.1%)**, 0 s at red; avg wait **21.25 -> 9.28 s (-56.3%)** |
| S6 | Hill-climbing tuning | unseen traffic: **8.65 s -> 6.96 s (-20%)** |
| S7 | Search on 6x6 congested grid, 200 trips | A\* **100% optimal**, 14.1 nodes vs UCS 36.0; Greedy only 51.5% optimal |
| S8 | 2x2 ... 8x8 grids (461 agents) | Max-Pressure -42..47% wait at every size; 1.5 ms per simulated second at 8x8 |

Honest negatives are reported too. The optional green-wave hand-off *slightly hurts* on random grid demand. In S5 single cars can be slower in our system; the gain is in the average and for the ambulance.

---

## Project layout

```
run_demo.py                the demo (writes + opens demo.html)
run_simulation.py          experiment tables (rich)
generate_results.py        runs every scenario -> results/
src/
  agents/                  base, intersection, vehicle, emergency (ambulance), coordinator
  algorithms/              search_astar.py (BFS/UCS/Greedy/A*/W-A*), max_pressure.py, hill_climbing.py
  environment/             road_network.py (BPR, saturation flow), traffic_grid.py (R x C), simulator.py
  protocols/message.py     FIPA-ACL messages + broker
  demo.py, demo_page.html  the re-route demo (data recorder + one-page viewer)
  visualization/charts.py  report figures
  scenarios.py             S1..S8 evaluation scenarios
tests/                     test_unit.py (23), test_integration.py (7), test_scenarios.py (8), test_demo.py (5)
results/                   results.json, figures/, astar_trace.txt, protocol_trace.txt, test_report.json
deliverables/              Report, Testing Document, Presentation
```

## Modelling assumptions (say these if asked)

- Time step 1 s; every road is one lane; a stop line discharges one vehicle every 2 s (1800 veh/h, the HCM saturation flow).
- Every intersection is a 4-leg, 2-phase signal (N-S / E-W through movements), with a 3 s yellow clearance on every switch.
- Road travel time follows BPR: `t = t0 (1 + 0.15 (V/C)^4)`.
- Vehicles plan on their *own beliefs*. They learn about incidents only from TMC broadcasts or by reaching the road themselves.
- Traffic queued in front of an ambulance is assumed to pull aside.
