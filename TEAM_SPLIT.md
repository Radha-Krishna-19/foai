# 5-person work split

Each member owns one part of the code, explains it in the review, and answers questions on it.
Pick the member numbers in any order; M1 drives the laptop during the demo.

| | Role | Code you own | Algorithm / idea you explain | Show in the demo | Tests you own | Report + slides |
|---|---|---|---|---|---|---|
| **M1** | Environment, simulator, demo | `src/environment/` (road_network, traffic_grid, simulator), `src/demo.py`, `src/demo_page.html`, `run_demo.py` | The world: road graph, BPR travel time, 1-second tick, grid scaling | Run `python run_demo.py` (conventional vs ours side by side), add and remove accident roads, scrub the timeline | UT-09..12, DM-01..03, S8 scalability | PEAS: Environment; Section 4 (tools); Demo slide |
| **M2** | Intersection agent | `src/agents/intersection_agent.py`, `src/algorithms/max_pressure.py` | Max-Pressure signal choice, min/max green, 3 s yellow, anti-starvation | Point at the lights: green goes to the road with more waiting cars; cars stop on red | UT-13..18, S1, S2 | PEAS: Intersection; Max-Pressure slides |
| **M3** | Vehicle agent and route search | `src/agents/vehicle_agent.py`, `src/algorithms/search_astar.py` | A* (heuristic, BPR cost), BFS/UCS/Greedy comparison, beliefs and re-planning | Click Car 1: original route (dashed) vs new route (solid) and the stated reason | UT-01..08, UT-22, UT-23, S4, S7 | Algorithmic Modeling; A* trace slide |
| **M4** | Ambulance agent | `src/agents/emergency_agent.py` | Signal preemption: ambulance looks ahead, sends PREEMPT_OVERRIDE, junction replies AGREE and turns green via yellow, ambulance sends CANCEL | `python run_simulation.py --protocol`, `--scenario 3`, `--scenario 5` (conventional vs ours) | IT-01, IT-02, DM-04, S3, S5 | Multi-agent interaction slides; message-flow figure; S5 table |
| **M5** | Coordinator, protocol, testing | `src/agents/coordinator_agent.py`, `src/protocols/message.py`, `src/algorithms/hill_climbing.py`, `tests/run_tests.py` | FIPA-ACL messages and broker, incident broadcast, hill-climbing tuning of signal parameters | `python tests/run_tests.py` (43/43), `--scenario 6` | IT-05..09, S6, test runner | Testing document; Q&A sheet |

## Live review order (about 8 minutes)
1. M1: PEAS and environment (1 min), open the demo and add an accident.
2. M3: click a re-routed car, explain A* and re-planning (2 min).
3. M2: lights and Max-Pressure (1.5 min).
4. M4: ambulance preemption messages and the conventional-vs-ours result table (1.5 min).
5. M5: incident broadcast, hill climbing, tests passing (2 min).

## Each member should be able to answer
- What does my agent sense, decide and do (its PEAS)?
- Which algorithm does it use and why not the obvious alternative?
- Which message does it send or receive?
- Which test proves it works, and what did the test find?
