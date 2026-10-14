# Work split (5 members)

Every member owns one AI component end to end: the agent or model, its algorithm, its tests,
its experiment, its section of the report and its slides. The website is only a viewer for the
simulation; nobody presents it as their own contribution.

The system uses three algorithms: **A\*** (M3), **Max-Pressure** (M2) and **Hill Climbing** (M5).

| | Component | AI topic | Code | Tests | Experiment |
|---|---|---|---|---|---|
| M1 | Task environment and simulation | PEAS, environment properties, stochastic demand model | `stms/environment/` | UT-09..12, IT-08, IT-09, DM-05, DM-07 | S8 scalability |
| M2 | Junction agent | Max-Pressure signal control (utility-based agent) | `stms/agents/intersection_agent.py`, `stms/algorithms/max_pressure.py` | UT-13..18, IT-02, IT-07, DM-06 | S1, S2 |
| M3 | Vehicle agent | A\* informed search, belief update and re-planning (goal-based agent) | `stms/agents/vehicle_agent.py`, `stms/algorithms/search_astar.py` | UT-01..08, UT-22, UT-23, IT-05, DM-01..03 | S4, S7 |
| M4 | Ambulance agent and messaging | FIPA-ACL interaction protocol, signal preemption | `stms/agents/emergency_agent.py`, `stms/protocols/message.py` | UT-19, UT-20, IT-01, DM-04 | S3, S5 |
| M5 | Traffic centre agent | Hill climbing with random restarts (local search), incident broadcast | `stms/agents/coordinator_agent.py`, `stms/algorithms/hill_climbing.py`, `tests/` | UT-21, IT-06, DM-08, DM-09 | S6 |

## What each member explains

**M1, task environment.** PEAS of the whole system and of each agent. Environment properties:
partially observable, stochastic, dynamic, sequential, multi-agent. Road graph G = (V, E), BPR
travel time t = t0 (1 + 0.15 (V/C)^4), saturation flow of one car per 2 s, Poisson vehicle
arrivals, the 1 s simulation loop and why the order sense, decide, act matters.

**M2, Max-Pressure.** Pressure of a phase = sum over movements of max(0, queue in - queue out).
The junction gives green to the phase with the higher score, with minimum and maximum green,
a 3 s yellow, an anti-starvation term and the fixed-time baseline it is compared with.

**M3, A\*.** f(n) = g(n) + h(n) with g = BPR travel time and h = straight-line distance / top speed.
Proof that h is admissible and consistent (tests UT-01, UT-02). How a vehicle updates its beliefs
from an accident alert and re-plans from the next junction. S7 compares A\* with BFS, UCS and
Greedy only as baselines to justify the choice.

**M4, ambulance and protocol.** FIPA-ACL message structure and the broker. Preemption protocol:
the ambulance looks ahead, sends PREEMPT_OVERRIDE, the junction replies AGREE and turns green
through yellow, the ambulance sends CANCEL once it has crossed. Result of S3 and S5.

**M5, hill climbing.** State = (min green, max green, starvation weight, switch threshold).
Neighbours = one step up or down in one parameter. Cost = average wait in simulation.
Random restarts against local minima, training and validation seeds, and how the traffic
centre pushes the best parameters to every junction with INFORM PARAMETER_UPDATE. Owner of the
test runner and the testing document.

## Review order (about 8 minutes)

1. M1: problem, PEAS, environment properties, start the demo.
2. M3: press Send car, show the A\* route, close a road on it, show the re-plan.
3. M2: point at a junction, read its Max-Pressure scores and decision.
4. M4: press Send ambulance, show the PREEMPT_OVERRIDE, AGREE, CANCEL messages and the ambulance times.
5. M5: hill-climbing parameters, then `python tests/run_tests.py`.

## Questions each member should be ready for

- What does my agent sense, decide and do?
- Why this algorithm and not the obvious alternative?
- Which message does it send or receive?
- Which test proves it works?
