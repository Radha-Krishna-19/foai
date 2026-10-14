"""
Discrete-time multi-agent traffic simulator (dt = 1 s).

Owner: Member 1 (Environment & Simulation)

Tick order (synchronous, deterministic for a given seed):
    1. Coordinator (TMC)   - telemetry, advisories
    2. Intersection agents - read queues + messages, decide phase, actuate lights
    3. Vehicle agents      - read lights / messages, move, cross, re-plan
    4. Metrics             - arrivals, queues, message counts
"""

from typing import Any, Callable, Dict, List, Optional, Tuple
import random

from stms.environment.road_network import RoadNetwork
from stms.agents.intersection_agent import IntersectionAgent
from stms.agents.vehicle_agent import VehicleAgent
from stms.agents.emergency_agent import EmergencyVehicleAgent
from stms.agents.coordinator_agent import CentralCoordinatorAgent
from stms.algorithms.search_astar import AStarRouter
from stms.protocols.message import MessageBroker


class SimulationMetrics:
    def __init__(self):
        self.total_spawned_vehicles = 0
        self.total_arrived_vehicles = 0
        self.total_wait_time = 0.0
        self.total_travel_time = 0.0
        self.total_fuel_liters = 0.0
        self.total_stops = 0
        self.max_wait_single_vehicle = 0.0
        self.emergency_travel_time: Optional[float] = None
        self.emergency_wait_time: Optional[float] = None
        self.max_observed_queue = 0
        self.total_messages_exchanged = 0
        self.queue_series: List[int] = []

    @property
    def average_wait_time(self) -> float:
        return self.total_wait_time / self.total_arrived_vehicles if self.total_arrived_vehicles else 0.0

    @property
    def average_travel_time(self) -> float:
        return self.total_travel_time / self.total_arrived_vehicles if self.total_arrived_vehicles else 0.0

    @property
    def throughput_rate(self) -> float:
        return 100.0 * self.total_arrived_vehicles / self.total_spawned_vehicles if self.total_spawned_vehicles else 0.0


class TrafficSimulator:
    def __init__(self, network: RoadNetwork, intersections: Dict[str, IntersectionAgent], broker: MessageBroker,
                 coordinator: Optional[CentralCoordinatorAgent] = None, router: Optional[AStarRouter] = None,
                 reroute_policy: str = "broadcast", record_positions: bool = False):
        self.network = network
        self.intersections = intersections
        self.broker = broker
        self.coordinator = coordinator or CentralCoordinatorAgent("TMC_COORD", broker, network)
        self.router = router or AStarRouter(network)
        self.reroute_policy = reroute_policy
        self.vehicles: List[VehicleAgent] = []
        self.emergency_vehicles: List[EmergencyVehicleAgent] = []
        self.sim_time = 0.0
        self.metrics = SimulationMetrics()
        self.record_positions = record_positions
        self.frames: List[Dict[str, Any]] = []
        self.events: List[Tuple[float, Callable[["TrafficSimulator"], None]]] = []

    # ---- spawning -------------------------------------------------------
    def _register(self, v: VehicleAgent):
        if v.reroute_policy == "broadcast":
            advisories = self.coordinator.current_advisories()
            if any([v.learn(a) for a in advisories]):
                v.replan_route()
                v.reroute_count = 0
        self.vehicles.append(v)
        self.metrics.total_spawned_vehicles += 1
        return v

    def spawn_vehicle(self, origin: str, destination: str, departure_time: Optional[float] = None) -> VehicleAgent:
        vid = f"VEH_{self.metrics.total_spawned_vehicles + 1:04d}"
        dep = self.sim_time if departure_time is None else departure_time
        return self._register(VehicleAgent(vid, self.broker, self.network, origin, destination, self.router,
                                           departure_time=dep, reroute_policy=self.reroute_policy))

    def spawn_emergency_vehicle(self, origin: str, destination: str, departure_time: Optional[float] = None,
                                preemption_enabled: bool = True) -> EmergencyVehicleAgent:
        eid = f"AMBULANCE_{len(self.emergency_vehicles) + 1:02d}"
        dep = self.sim_time if departure_time is None else departure_time
        ev = EmergencyVehicleAgent(eid, self.broker, self.network, origin, destination, self.router,
                                   departure_time=dep, preemption_enabled=preemption_enabled)
        self.emergency_vehicles.append(ev)
        return self._register(ev)

    def spawn_poisson_demand(self, od_pairs: List[Tuple[str, str]], rate_per_s: float, duration: float,
                             rng: random.Random, weights: Optional[List[float]] = None) -> int:
        """Poisson arrivals: exponential inter-arrival times with mean 1/rate."""
        t, n = 0.0, 0
        while True:
            t += rng.expovariate(rate_per_s)
            if t > duration:
                return n
            o, d = rng.choices(od_pairs, weights=weights)[0]
            self.spawn_vehicle(o, d, departure_time=round(t))
            n += 1

    def schedule(self, at_time: float, fn: Callable[["TrafficSimulator"], None]):
        self.events.append((at_time, fn))

    # ---- main loop -------------------------------------------------------
    def env_state(self) -> Dict[str, Any]:
        return {"time": self.sim_time, "network": self.network, "segments": self.network.segments,
                "intersections": self.intersections,
                "vehicle_map": {v.agent_id: v for v in self.vehicles if v.departed and not v.has_arrived}}

    def step(self):
        self.sim_time += 1.0
        self.broker.sim_time = self.sim_time
        for t, fn in [e for e in self.events if e[0] == self.sim_time]:
            fn(self)
        env = self.env_state()

        self.coordinator.step(env)
        for inter in self.intersections.values():
            inter.step(env)
        for v in self.vehicles:
            if not v.has_arrived:
                v.step(env)

        q = max((len(s.vehicles) for s in self.network.segments.values()), default=0)
        self.metrics.max_observed_queue = max(self.metrics.max_observed_queue, q)
        self.metrics.queue_series.append(sum(len(s.vehicles) for s in self.network.segments.values()))

        for v in self.vehicles:
            if v.has_arrived and v.active:
                v.active = False
                self.broker.unregister_agent(v.agent_id)
                m = self.metrics
                m.total_arrived_vehicles += 1
                m.total_wait_time += v.total_wait_time
                m.total_travel_time += v.total_travel_time
                m.total_fuel_liters += v.fuel_consumed_liters
                m.total_stops += v.stops
                m.max_wait_single_vehicle = max(m.max_wait_single_vehicle, v.total_wait_time)
                if isinstance(v, EmergencyVehicleAgent):
                    m.emergency_travel_time = v.total_travel_time
                    m.emergency_wait_time = v.total_wait_time
        self.metrics.total_messages_exchanged = self.broker.total_messages_dispatched

        if self.record_positions:
            self.frames.append(self.snapshot())

    def snapshot(self) -> Dict[str, Any]:
        return {
            "t": self.sim_time,
            "phases": {k: a.current_phase.value for k, a in self.intersections.items()},
            "vehicles": [(v.agent_id, *self._drawn_xy(v), v.agent_type.value) for v in self.vehicles
                         if v.departed and not v.has_arrived],
            "queue_total": sum(len(s.vehicles) for s in self.network.segments.values()),
        }

    def _drawn_xy(self, v, offset: float = 9.0):
        """Vehicle position shifted to the LEFT of its direction of travel (India drives on the left)."""
        x, y = v.xy()
        seg = v.current_segment
        if seg is None:
            return x, y
        (x1, y1), (x2, y2) = self.network.node_positions[seg.source], self.network.node_positions[seg.target]
        L = max(1e-6, ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5)
        dx, dy = (x2 - x1) / L, (y2 - y1) / L
        return x - dy * offset, y + dx * offset

    def run(self, duration_seconds: int, step_callback=None):
        for tick in range(1, int(duration_seconds) + 1):
            self.step()
            if step_callback:
                step_callback(self, tick)

    def all_done(self) -> bool:
        return all(v.has_arrived for v in self.vehicles)

    def run_until_clear(self, max_time: int = 3600, min_time: int = 0) -> bool:
        while self.sim_time < max_time:
            self.step()
            if self.sim_time >= min_time and self.all_done():
                return True
        return self.all_done()

    # ---- reporting -------------------------------------------------------
    def summary(self, only=None) -> Dict[str, Any]:
        """Per-vehicle statistics. `only` = optional predicate selecting vehicles (e.g. commuters only)."""
        vs = [v for v in self.vehicles if (only is None or only(v))]
        done = [v for v in vs if v.has_arrived]
        waits = sorted(v.total_wait_time for v in done)
        n = len(done)
        return {
            "spawned": len(vs),
            "arrived": n,
            "unfinished": len(vs) - n,
            "avg_wait_s": round(sum(waits) / n, 2) if n else None,
            "p95_wait_s": round(waits[min(n - 1, int(0.95 * n))], 1) if n else None,
            "max_wait_s": round(waits[-1], 1) if n else None,
            "avg_travel_s": round(sum(v.total_travel_time for v in done) / n, 2) if n else None,
            "avg_stops": round(sum(v.stops for v in done) / n, 2) if n else None,
            "fuel_l": round(sum(v.fuel_consumed_liters for v in vs), 3),
            "max_queue": self.metrics.max_observed_queue,
            "messages": self.broker.total_messages_dispatched,
            "sim_time_s": self.sim_time,
        }
