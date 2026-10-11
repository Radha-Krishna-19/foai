"""
Steepest-descent Hill Climbing with random restarts - used by the Coordinator (TMC) to tune
the Max-Pressure controller parameters of every intersection.

Owner: Member 5 (Coordinator Agent & Optimisation)

Local-search formulation
------------------------
    State      : x = (min_green, max_green, starvation_weight, switch_threshold)   (discrete grid)
    Neighbours : change ONE parameter by one grid step up or down   (<= 8 neighbours)
    Objective  : J(x) = average vehicle delay (s) in a simulation with a fixed demand seed
                 (lower is better)
    Move       : go to the best neighbour if it improves J, otherwise stop (local optimum)
    Restarts   : repeat from random states to escape local optima; keep the global best
Complexity: each iteration costs <= 8 simulations; results are memoised so revisited
states are free.
"""

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Sequence, Tuple
import random

State = Tuple[int, ...]   # indices into each parameter's value grid


@dataclass
class HillClimbResult:
    best_state: State
    best_cost: float
    best_params: Dict[str, float]
    evaluations: int
    trace: List[Dict] = field(default_factory=list)   # one row per iteration


class ParameterSpace:
    def __init__(self, grid: Dict[str, Sequence[float]]):
        self.names = list(grid)
        self.values = [list(v) for v in grid.values()]

    def to_params(self, s: State) -> Dict[str, float]:
        return {n: self.values[i][s[i]] for i, n in enumerate(self.names)}

    def neighbours(self, s: State) -> List[State]:
        out = []
        for i in range(len(s)):
            for d in (-1, 1):
                j = s[i] + d
                if 0 <= j < len(self.values[i]):
                    t = list(s)
                    t[i] = j
                    out.append(tuple(t))
        return out

    def random_state(self, rng: random.Random) -> State:
        return tuple(rng.randrange(len(v)) for v in self.values)

    def index_of(self, params: Dict[str, float]) -> State:
        return tuple(min(range(len(self.values[i])), key=lambda k: abs(self.values[i][k] - params[n]))
                     for i, n in enumerate(self.names))


SIGNAL_PARAMETER_GRID = {
    "min_green_time":    [3, 4, 5, 6, 8, 10, 12],
    "max_green_time":    [15, 20, 25, 30, 40, 50, 60],
    "starvation_weight": [0.0, 0.1, 0.25, 0.5, 0.75, 1.0, 1.5],
    "switch_threshold":  [0.0, 0.5, 1.0, 2.0, 3.0, 4.0],
}


def hill_climb(space: ParameterSpace, cost_fn: Callable[[Dict[str, float]], float],
               start: State, restarts: int = 2, max_iters: int = 15, seed: int = 7) -> HillClimbResult:
    rng = random.Random(seed)
    cache: Dict[State, float] = {}
    trace: List[Dict] = []

    def J(s: State) -> float:
        if s not in cache:
            cache[s] = cost_fn(space.to_params(s))
        return cache[s]

    best_s, best_c = start, J(start)
    starts = [start] + [space.random_state(rng) for _ in range(restarts)]
    for r, s in enumerate(starts):
        cur, cur_c = s, J(s)
        trace.append({"restart": r, "iter": 0, "params": space.to_params(cur), "cost": round(cur_c, 3),
                      "move": "start"})
        for it in range(1, max_iters + 1):
            nbrs = space.neighbours(cur)
            scored = [(J(n), n) for n in nbrs]
            c, n = min(scored)
            if c < cur_c - 1e-9:
                cur, cur_c = n, c
                trace.append({"restart": r, "iter": it, "params": space.to_params(cur), "cost": round(cur_c, 3),
                              "move": "improve"})
            else:
                trace.append({"restart": r, "iter": it, "params": space.to_params(cur), "cost": round(cur_c, 3),
                              "move": "local optimum"})
                break
        if cur_c < best_c:
            best_s, best_c = cur, cur_c
    return HillClimbResult(best_s, best_c, space.to_params(best_s), len(cache), trace)
