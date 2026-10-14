# STMS: Smart Traffic Management System

A multi-agent AI system for city traffic. Junctions, vehicles, an ambulance and a traffic centre
are independent agents that sense, decide and act on their own and coordinate through FIPA-ACL
messages. The system is compared with a conventional fixed-time city under identical traffic.

**Website:** open `index.html`, or run `python run_demo.py`. Send a car or an ambulance, close
roads with accidents, and watch both cities side by side.

## Agents and algorithms

| Agent | Decides | Algorithm | Messages |
|---|---|---|---|
| Junction | which direction gets green | **Max-Pressure** with anti-starvation | AGREE, green-wave INFORM |
| Vehicle | which route to drive | **A\*** search, re-planned after accident alerts | receives INCIDENT_ALERT |
| Ambulance | when to ask for priority | look-ahead preemption | PREEMPT_OVERRIDE, CANCEL |
| Traffic centre | signal timing, incident alerts | **Hill climbing** with random restarts | BROADCAST, INFORM PARAMETER_UPDATE |

## Results

All numbers are produced by `python scripts/generate_results.py` and stored in `results/results.json`.

| Experiment | Result |
|---|---|
| S1 Max-Pressure vs fixed-time, 2x2 grid, 10 seeds | average wait 15.45 s to 8.11 s (-47.5%) |
| S2 Rush hour, 87% north-south demand | anti-starvation leaves 0 cross-street cars unserved (14 without it) |
| S3 Ambulance with and without preemption | 54.0 s to 38.0 s (-29.6%), 0 s at red, no cost to other traffic |
| S4 Closed road | fixed routes: 0% arrive; A\* re-routing: 100% arrive |
| S5 Conventional city vs STMS, 3x3, dense traffic, accident | ambulance 119 s to 64 s (-46%), average wait 24.3 s to 16.7 s (-31%) |
| S6 Hill-climbing tuning | average wait on unseen traffic 8.65 s to 6.96 s (-20%) |
| S7 A\* against baselines, 6x6 grid, 200 trips | A\* 100% optimal with 14.1 nodes expanded (UCS 36.0) |
| S8 Scalability 2x2 to 8x8 | Max-Pressure 42-47% less wait at every size |

Reported limitations: the green-wave hand-off slightly increases wait on random demand (S1), and a
single car can occasionally be slower in STMS; the gain is in the average and for the ambulance.

## Running

Python 3.9 or newer. The simulation itself needs no extra packages.

```
python run_demo.py                      # website on http://localhost:8000
python tests/run_tests.py               # 47 automated tests -> results/test_report.json
python scripts/run_simulation.py --all  # every experiment as a table
python scripts/generate_results.py      # results.json, figures, traces and index.html
python scripts/build_site.py            # rebuild index.html only
```

On Windows, double-click `START_DEMO.bat`. The charts and console tables need
`pip install -r requirements.txt` (matplotlib, rich).

`index.html` also works on its own (for example on GitHub Pages): without the local server it
runs the same Python package in the browser with Pyodide. That needs an internet connection the
first time; `run_demo.py` works offline.

## Repository layout

```
index.html               website (generated)
run_demo.py              local server for the website
stms/
  agents/                junction, vehicle, ambulance, traffic centre
  algorithms/            search_astar.py, max_pressure.py, hill_climbing.py
  environment/           road network (BPR), grid builder, simulator
  protocols/             FIPA-ACL messages and broker
  scenarios.py           experiments S1-S8
  demo.py                records the two-city run shown on the website
  site.py                builds index.html
  visualization/         report figures
web/template.html        website source
scripts/                 generate_results.py, run_simulation.py, build_site.py
tests/                   unit, integration, scenario and demo tests
results/                 results.json, figures, traces, test report
docs/                    report, testing document, presentation, team split
```

## Team

| Member | Component | Algorithm |
|---|---|---|
| M1 P.M. Radha Krishna (CB.SC.U4CSE23134) | task environment and simulation | PEAS, stochastic demand model |
| M2 | junction agent | Max-Pressure |
| M3 | vehicle agent | A\* |
| M4 | ambulance agent and FIPA-ACL protocol | preemption protocol |
| M5 | traffic centre agent and testing | hill climbing |

Details in [docs/TEAM_SPLIT.md](docs/TEAM_SPLIT.md).

## Modelling assumptions

- 1 s time step; single-lane roads; a stop line releases one vehicle every 2 s (1800 veh/h).
- Four-leg junctions with two phases (north-south, east-west) and a 3 s yellow on every change.
- Travel time follows BPR: t = t0 (1 + 0.15 (V/C)^4).
- Vehicles plan on their own beliefs and learn about accidents only from the traffic centre or on arrival.
- Traffic in front of an ambulance pulls aside.
