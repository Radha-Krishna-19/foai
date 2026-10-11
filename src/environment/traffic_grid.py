"""
Grid network generators.

build_grid_network(rows, cols) creates an R x C grid of signalised intersections with
entry/exit feeder nodes on every boundary road. build_2x2_grid_network() is the
default demo network used in Review 1.

              N0          N1
              |           |
      W0 -- INT_00 ---- INT_01 -- E0
              |           |
      W1 -- INT_10 ---- INT_11 -- E1
              |           |
              S0          S1

Owner: Member 1 (Environment & Simulation)
"""

from typing import Dict, Tuple
from src.environment.road_network import RoadNetwork
from src.agents.intersection_agent import IntersectionAgent
from src.protocols.message import MessageBroker


def _int_id(r: int, c: int, big: bool) -> str:
    return f"INT_{r}_{c}" if big else f"INT_{r}{c}"


def build_grid_network(broker: MessageBroker, rows: int = 2, cols: int = 2, mode: str = "MAX_PRESSURE",
                       spacing: float = 200.0, capacity: int = 20, feeder_capacity: int = 25,
                       controller_params: Dict = None, green_wave: bool = True
                       ) -> Tuple[RoadNetwork, Dict[str, IntersectionAgent]]:
    big = rows > 10 or cols > 10
    net = RoadNetwork()
    top = (rows - 1) * spacing

    def pos(r, c):
        return c * spacing, top - r * spacing

    # intersections + feeders
    for r in range(rows):
        for c in range(cols):
            x, y = pos(r, c)
            net.add_intersection(_int_id(r, c, big), x, y)
    for c in range(cols):
        net.add_intersection(f"N{c}", c * spacing, top + spacing)
        net.add_intersection(f"S{c}", c * spacing, -spacing)
    for r in range(rows):
        net.add_intersection(f"W{r}", -spacing, top - r * spacing)
        net.add_intersection(f"E{r}", cols * spacing, top - r * spacing)

    def both(a, b, cap):
        net.add_road(a, b, length_meters=spacing, capacity=cap)
        net.add_road(b, a, length_meters=spacing, capacity=cap)

    for r in range(rows):
        for c in range(cols):
            me = _int_id(r, c, big)
            if c + 1 < cols:
                both(me, _int_id(r, c + 1, big), capacity)
            if r + 1 < rows:
                both(me, _int_id(r + 1, c, big), capacity)
    for c in range(cols):
        both(f"N{c}", _int_id(0, c, big), feeder_capacity)
        both(f"S{c}", _int_id(rows - 1, c, big), feeder_capacity)
    for r in range(rows):
        both(f"W{r}", _int_id(r, 0, big), feeder_capacity)
        both(f"E{r}", _int_id(r, cols - 1, big), feeder_capacity)

    # intersection agents
    agents: Dict[str, IntersectionAgent] = {}
    for r in range(rows):
        for c in range(cols):
            me = _int_id(r, c, big)
            north = _int_id(r - 1, c, big) if r > 0 else f"N{c}"
            south = _int_id(r + 1, c, big) if r + 1 < rows else f"S{c}"
            west = _int_id(r, c - 1, big) if c > 0 else f"W{r}"
            east = _int_id(r, c + 1, big) if c + 1 < cols else f"E{r}"
            agent = IntersectionAgent(
                agent_id=me, broker=broker, controlled_node=me,
                ns_incoming_roads=[f"{north}->{me}", f"{south}->{me}"],
                ns_outgoing_roads=[f"{me}->{north}", f"{me}->{south}"],
                ew_incoming_roads=[f"{west}->{me}", f"{east}->{me}"],
                ew_outgoing_roads=[f"{me}->{west}", f"{me}->{east}"],
                mode=mode, controller_params=controller_params, green_wave=green_wave,
            )
            agents[me] = agent
    # neighbour links for the green-wave hand-off (axis = direction of travel between them)
    for r in range(rows):
        for c in range(cols):
            me = _int_id(r, c, big)
            if r > 0:
                agents[me].add_neighbor(_int_id(r - 1, c, big), "NS")
            if r + 1 < rows:
                agents[me].add_neighbor(_int_id(r + 1, c, big), "NS")
            if c > 0:
                agents[me].add_neighbor(_int_id(r, c - 1, big), "EW")
            if c + 1 < cols:
                agents[me].add_neighbor(_int_id(r, c + 1, big), "EW")
    return net, agents


def build_2x2_grid_network(broker: MessageBroker, mode: str = "MAX_PRESSURE", **kw):
    return build_grid_network(broker, 2, 2, mode=mode, **kw)
