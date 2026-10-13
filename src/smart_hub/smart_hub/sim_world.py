"""
Deterministic 2D world used by scenario_runner (assignment 3.2). Owner: M8.

Pure Python (no rclpy): fixed time step, seeded randomness, timed scenario events,
unicycle kinematics with swept collision checks, private ground truth and noisy
observations. The ROS node only feeds commands in and publishes what this class reports.
"""
from dataclasses import dataclass, field
import math
import random

from smart_hub import hub_map

STEP_SEC = 0.05
ROBOT_RADIUS = 0.20
VEHICLE_RADIUS = 0.30
PEDESTRIAN_RADIUS = 0.20
COMMAND_TIMEOUT_SEC = 0.5
SWEEP_STEP_M = 0.05
EDGE_CORRIDOR_M = 0.5
BOARDING_CAPACITY = 4
BOARDING_DWELL_SEC = 8.0
DOCK_TOLERANCE_M = 0.20
STOPPED_SPEED_MPS = 0.02
CROSSING_PEDESTRIAN_POS = (5.0, 3.0)

# Event types that the ROS node executes as service / action calls (test driver).
DRIVER_EVENTS = {
    'reserve', 'release', 'dock_request', 'escort_request', 'confirm_ready',
    'clear_stop_request', 'cancel_goal',
}


@dataclass
class Agent:
    agent_id: str
    kind: str                   # bus, car, robot, obstacle
    x: float
    y: float
    yaw: float = 0.0
    radius: float = VEHICLE_RADIUS
    v: float = 0.0
    w: float = 0.0
    controlled: bool = False    # True: moved by final cmd_vel; False: scripted
    cmd_time: float = -math.inf
    route: list = field(default_factory=list)   # scripted cars: node ids, looped
    route_index: int = 0
    speed: float = 0.0          # scripted cars: cruise speed
    frozen: bool = False        # observation source frozen (stale stamp)
    in_collision: str = ''


@dataclass
class Pedestrian:
    ped_id: str
    x: float
    y: float
    zone: str                   # Q or C
    entered_at: float
    leave_at: float = math.inf
    occluded_until: float = -math.inf


@dataclass
class WorldEvent:
    t: float
    entity_id: str
    event: str
    reason: str = ''
    correlation_id: str = ''


class World:
    """One seeded trial. Create a new instance to reset all state."""

    def __init__(self, scenario, seed):
        self.scenario = scenario or {}
        self.seed = int(seed)
        self.rng = random.Random(self.seed)
        self.t = 0.0
        self.duration = float(self.scenario.get('duration_sec', math.inf))
        self.events = sorted(self.scenario.get('events', []), key=lambda e: float(e['t']))
        self._next_event = 0

        self.agents = {}
        self.peds = {}
        self._ped_counter = 0
        self.arrival_rate_per_min = 0.0

        self.slots = {s: False for s in hub_map.PARKING_SLOTS}      # truth: occupied?
        self.slot_connected = {s: True for s in hub_map.PARKING_SLOTS}
        self.reservations = {}                                      # client -> slot id

        self.camera_frozen_until = -math.inf
        self.odom_frozen_until = -math.inf
        self.estop = False
        self.door_open = False
        self._boarded_this_stop = 0
        self._next_boarding_t = math.inf

        self.log = []               # WorldEvent list, drained by the node
        self.requests = []          # driver events, drained by the node

        self._spawn_defaults()
        for slot, state in self.scenario.get('initial_slots', {}).items():
            self.slots[slot] = str(state).upper() == 'OCCUPIED'
        for car in self.scenario.get('initial_vehicles', []):
            self._ev_spawn_car(car)

    # ------------------------------------------------------------------ setup
    def _spawn_defaults(self):
        bus = self.scenario.get('bus', {})
        bx, by = hub_map.NODES[bus.get('start_node', 'A')]
        self.agents['bus1'] = Agent('bus1', 'bus', bx, by, math.pi / 2, VEHICLE_RADIUS,
                                    controlled=True)
        robot = self.scenario.get('robot', {})
        rx, ry = hub_map.NODES[robot.get('start_node', 'R')]
        self.agents['robot1'] = Agent('robot1', 'robot', rx, ry, 0.0, ROBOT_RADIUS,
                                      controlled=True)

    # ------------------------------------------------------------------ inputs
    def command(self, agent_id, v, w):
        """Apply the most recent final velocity for a controlled agent."""
        a = self.agents.get(agent_id)
        if a is not None and a.controlled:
            a.v, a.w, a.cmd_time = float(v), float(w), self.t

    def set_door_open(self, is_open):
        if is_open and not self.door_open:
            self._boarded_this_stop = 0
            self._next_boarding_t = self.t + BOARDING_DWELL_SEC / BOARDING_CAPACITY
        self.door_open = bool(is_open)

    # ------------------------------------------------------------------ main loop
    def step(self):
        self._process_events()
        self._arrivals()
        self._expire_pedestrians()
        for a in self.agents.values():
            if a.controlled:
                self._move_controlled(a)
            elif a.kind == 'car':
                self._move_scripted(a)
        self._boarding()
        self.t = round(self.t + STEP_SEC, 6)

    def finished(self):
        return self.t >= self.duration

    def _emit(self, entity_id, event, reason='', correlation_id=''):
        self.log.append(WorldEvent(self.t, entity_id, event, reason, correlation_id))

    # ------------------------------------------------------------------ events
    def _process_events(self):
        while self._next_event < len(self.events) and \
                float(self.events[self._next_event]['t']) <= self.t + 1e-9:
            ev = dict(self.events[self._next_event])
            self._next_event += 1
            kind = ev['type']
            if kind in DRIVER_EVENTS:
                self.requests.append(ev)
                continue
            handler = getattr(self, '_ev_' + kind, None)
            if handler is None:
                self._emit('scenario_runner', 'UNKNOWN_EVENT', kind)
                continue
            handler(ev)
            self._emit('scenario_runner', 'INJECT', kind)

    def _new_ped(self, zone, x, y, leave_at=math.inf):
        self._ped_counter += 1
        pid = 'ped%03d' % self._ped_counter
        self.peds[pid] = Pedestrian(pid, x, y, zone, self.t, leave_at)
        return pid

    def _random_queue_point(self):
        """Pick a free spot in Q, off the robot corridor R-W (y = 6) and inside a margin."""
        xmin, xmax, ymin, ymax = hub_map.QUEUE_Q
        corridor_y = hub_map.NODES['R'][1]
        for _ in range(200):
            x = self.rng.uniform(xmin + 0.3, xmax - 0.3)
            if self.rng.random() < 0.5:
                y = self.rng.uniform(ymin + 0.3, corridor_y - 0.5)
            else:
                y = self.rng.uniform(corridor_y + 0.5, ymax - 0.3)
            if all(math.hypot(x - p.x, y - p.y) > 2 * PEDESTRIAN_RADIUS + 0.05
                   for p in self.peds.values()):
                return x, y
        return x, y

    def queue_peds(self):
        return sorted((p for p in self.peds.values() if p.zone == 'Q'),
                      key=lambda p: p.entered_at)

    def _ev_spawn_queue_tracks(self, ev):
        for _ in range(int(ev.get('count', 1))):
            self._new_ped('Q', *self._random_queue_point())

    def _ev_remove_queue_tracks(self, ev):
        for p in self.queue_peds()[:int(ev.get('count', 1))]:
            del self.peds[p.ped_id]
            self._emit(p.ped_id, 'LEFT_QUEUE', 'scenario')

    def _ev_fill_queue(self, ev):
        missing = int(ev['target']) - len(self.queue_peds())
        self._ev_spawn_queue_tracks({'count': max(0, missing)})

    def _ev_arrival_rate(self, ev):
        self.arrival_rate_per_min = float(ev['per_min'])

    def _ev_crossing_pedestrian(self, ev):
        x, y = ev.get('position', CROSSING_PEDESTRIAN_POS)
        self._new_ped('C', x, y, self.t + float(ev.get('duration_sec', 4.0)))

    def _ev_occlude(self, ev):
        for p in self.queue_peds()[:int(ev.get('count', 1))]:
            p.occluded_until = self.t + float(ev['duration_sec'])

    def _ev_freeze_camera(self, ev):
        self.camera_frozen_until = self.t + float(ev.get('duration_sec', math.inf))

    def _ev_freeze_odometry(self, ev):
        self.odom_frozen_until = self.t + float(ev.get('duration_sec', math.inf))

    def _ev_freeze_vehicle_source(self, ev):
        self.agents[ev['vehicle']].frozen = True

    def _ev_restore_sensors(self, ev):
        self.camera_frozen_until = -math.inf
        self.odom_frozen_until = -math.inf
        for a in self.agents.values():
            a.frozen = False
        for s in self.slot_connected:
            self.slot_connected[s] = True

    def _ev_place_obstacle(self, ev):
        x, y = ev['center']
        self.agents[ev['id']] = Agent(ev['id'], 'obstacle', float(x), float(y),
                                      radius=float(ev.get('radius', 0.35)))

    def _ev_remove_obstacle(self, ev):
        a = self.agents.get(ev['id'])
        if a is not None and a.kind == 'obstacle':
            del self.agents[ev['id']]

    def _ev_block_edges(self, ev):
        for eid in ev['edges']:
            a, b = eid.split('-')
            (x0, y0), (x1, y1) = hub_map.NODES[a], hub_map.NODES[b]
            centre = ((x0 + x1) / 2, (y0 + y1) / 2)
            self._ev_place_obstacle({'id': 'block_' + eid, 'center': centre,
                                     'radius': ev.get('radius', 0.35)})

    def _ev_restore_all(self, ev):
        for aid in [k for k, a in self.agents.items() if a.kind == 'obstacle']:
            del self.agents[aid]
        self._ev_restore_sensors(ev)

    def _ev_spawn_car(self, ev):
        route = list(ev['route'])
        x, y = hub_map.NODES[route[0]]
        nx, ny = hub_map.NODES[route[1]]
        self.agents[ev['id']] = Agent(
            ev['id'], 'car', x, y, math.atan2(ny - y, nx - x), VEHICLE_RADIUS,
            route=route, route_index=1, speed=float(ev.get('speed_mps', 0.35)),
            v=float(ev.get('speed_mps', 0.35)))
        if 'start_fraction' in ev:
            f = float(ev['start_fraction'])
            a = self.agents[ev['id']]
            a.x, a.y = x + f * (nx - x), y + f * (ny - y)

    def _ev_spawn_slow_vehicles(self, ev):
        a, b = ev['edge'].split('-')
        count = int(ev.get('count', 1))
        for i in range(count):
            self._ev_spawn_car({
                'id': '%s_slow%d' % (ev['edge'], i), 'route': [a, b],
                'speed_mps': ev.get('speed_mps', 0.10),
                'start_fraction': (i + 0.5) / count * 0.8,
            })

    def _ev_set_slot(self, ev):
        self.slots[ev['slot']] = bool(ev['occupied'])

    def _ev_disconnect_slot_sensor(self, ev):
        self.slot_connected[ev['slot']] = False

    def _ev_reconnect_slot_sensor(self, ev):
        self.slot_connected[ev['slot']] = True

    def _ev_occupy_reserved_slot(self, ev):
        slot = self.reservations.get(ev['client'])
        if slot:
            self.slots[slot] = True
        else:
            self._emit(ev['client'], 'NO_RESERVATION', 'occupy_reserved_slot ignored')

    def _ev_estop(self, ev):
        self.estop = bool(ev['value'])

    # ------------------------------------------------------------------ dynamics
    def _arrivals(self):
        if self.arrival_rate_per_min > 0.0 and \
                self.rng.random() < self.arrival_rate_per_min / 60.0 * STEP_SEC:
            self._new_ped('Q', *self._random_queue_point())

    def _expire_pedestrians(self):
        for pid in [k for k, p in self.peds.items() if p.leave_at <= self.t]:
            del self.peds[pid]

    def _obstructions(self, agent):
        """Yield (id, x, y, radius) of everything the agent may collide with."""
        for o in self.agents.values():
            if o is not agent:
                yield o.agent_id, o.x, o.y, o.radius
        for p in self.peds.values():
            yield p.ped_id, p.x, p.y, PEDESTRIAN_RADIUS

    def _collides(self, agent, x, y):
        for oid, ox, oy, r in self._obstructions(agent):
            if math.hypot(x - ox, y - oy) < agent.radius + r:
                return oid
        return ''

    def _move_controlled(self, a):
        if self.t - a.cmd_time > COMMAND_TIMEOUT_SEC:
            a.v = a.w = 0.0
        if a.v == 0.0 and a.w == 0.0:
            return
        yaw = a.yaw + a.w * STEP_SEC
        dist = a.v * STEP_SEC
        n = max(1, math.ceil(abs(dist) / SWEEP_STEP_M))
        x, y = a.x, a.y
        for i in range(1, n + 1):
            heading = a.yaw + a.w * STEP_SEC * i / n
            nx = x + dist / n * math.cos(heading)
            ny = y + dist / n * math.sin(heading)
            hit = self._collides(a, nx, ny)
            if hit:
                if a.in_collision != hit:
                    self._emit(a.agent_id, 'COLLISION', 'contact with ' + hit)
                a.in_collision = hit
                a.v = 0.0
                break
            x, y = nx, ny
        else:
            a.in_collision = ''
        a.x, a.y = x, y
        a.yaw = math.atan2(math.sin(yaw), math.cos(yaw))

    def _move_scripted(self, a):
        tx, ty = hub_map.NODES[a.route[a.route_index]]
        dx, dy = tx - a.x, ty - a.y
        remaining = math.hypot(dx, dy)
        step = min(a.speed * STEP_SEC, remaining)
        nx = a.x + step * dx / remaining if remaining > 0 else a.x
        ny = a.y + step * dy / remaining if remaining > 0 else a.y
        # Keep headway: stop behind anything ahead within a small gap.
        if self._collides(a, nx + 0.1 * dx / max(remaining, 1e-9),
                          ny + 0.1 * dy / max(remaining, 1e-9)):
            a.v = 0.0
            return
        a.x, a.y, a.v = nx, ny, step / STEP_SEC
        a.yaw = math.atan2(dy, dx) if remaining > 0 else a.yaw
        if remaining - step < 1e-6:
            if a.route_index + 1 < len(a.route):
                a.route_index += 1
                return
            # Loop: re-enter at the route start (when free) to keep a steady flow.
            sx, sy = hub_map.NODES[a.route[0]]
            if self._collides(a, sx, sy):
                a.v = 0.0
                return
            a.x, a.y, a.route_index = sx, sy, 1

    def bus_docked(self):
        bus = self.agents['bus1']
        bx, by = hub_map.NODES[hub_map.BUS_BAY]
        return math.hypot(bus.x - bx, bus.y - by) <= DOCK_TOLERANCE_M and \
            abs(bus.v) < STOPPED_SPEED_MPS

    def _boarding(self):
        if not (self.door_open and self.bus_docked()):
            return
        if self._boarded_this_stop >= BOARDING_CAPACITY or self.t < self._next_boarding_t:
            return
        queue = self.queue_peds()
        if not queue:
            return
        p = queue[0]
        del self.peds[p.ped_id]
        self._boarded_this_stop += 1
        self._next_boarding_t = self.t + BOARDING_DWELL_SEC / BOARDING_CAPACITY
        self._emit(p.ped_id, 'BOARDED', 'wait_sec=%.2f' % (self.t - p.entered_at), 'bus1')

    # ------------------------------------------------------------------ observations
    def camera_frozen(self):
        return self.t < self.camera_frozen_until

    def odom_frozen(self):
        return self.t < self.odom_frozen_until

    def observed_pedestrians(self, noise_m):
        """Noisy foot-points of visible pedestrians; None while the camera is frozen."""
        if self.camera_frozen():
            return None
        out = []
        for p in sorted(self.peds.values(), key=lambda q: q.ped_id):
            if self.t < p.occluded_until:
                continue
            out.append((p.x + self.rng.gauss(0.0, noise_m), p.y + self.rng.gauss(0.0, noise_m)))
        return out

    def truth_pedestrians(self):
        peds = sorted(self.peds.values(), key=lambda q: q.ped_id)
        return [(p.ped_id, p.x, p.y, p.zone) for p in peds]

    def edge_of(self, agent):
        if agent.kind == 'obstacle':
            edges = hub_map.VEHICLE_EDGES + hub_map.ROBOT_EDGES
        else:
            edges = hub_map.allowed_edges('car' if agent.kind == 'bus' else agent.kind)
        best, best_d = '', EDGE_CORRIDOR_M
        for a, b in edges:
            d = _point_segment_distance((agent.x, agent.y), hub_map.NODES[a], hub_map.NODES[b])
            if d < best_d - 1e-9:
                best, best_d = hub_map.edge_id(a, b), d
        return best

    def observed_vehicles(self, noise_m):
        """Return (agent, x, y, yaw, speed, edge_id) with noise; frozen flag on agent."""
        out = []
        for a in sorted(self.agents.values(), key=lambda q: q.agent_id):
            noise = 0.0 if a.kind == 'obstacle' else noise_m
            out.append((a, a.x + self.rng.gauss(0.0, noise), a.y + self.rng.gauss(0.0, noise),
                        a.yaw, abs(a.v), self.edge_of(a)))
        return out

    def observed_slots(self):
        return {s: occ for s, occ in self.slots.items() if self.slot_connected[s]}


def _point_segment_distance(p, a, b):
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))
