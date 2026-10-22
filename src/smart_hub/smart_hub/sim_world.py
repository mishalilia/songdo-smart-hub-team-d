# Author: Mikhail Li (12236295)
"""
Deterministic 2D world used by scenario_runner (assignment 3.2).

Pure Python (no rclpy): fixed time step, seeded randomness, timed scenario events,
unicycle kinematics with swept collision checks, private ground truth and noisy
observations. The ROS node only feeds commands in and publishes what this class reports.
"""
from dataclasses import dataclass, field
import math
import random

from smart_hub import hub_map
from smart_hub.iot_devices import Barrier, Door, Led, Sensor

STEP_SEC = 0.05
ROBOT_RADIUS = 0.20
VEHICLE_RADIUS = 0.30
PEDESTRIAN_RADIUS = 0.20
COMMAND_TIMEOUT_SEC = 0.5
SWEEP_STEP_M = 0.05
EDGE_CORRIDOR_M = 0.5
NODE_RADIUS_M = 0.3
BOARDING_CAPACITY = 4
BOARDING_DWELL_SEC = 8.0
DOCK_TOLERANCE_M = 0.20
STOPPED_SPEED_MPS = 0.02
CROSSING_PEDESTRIAN_POS = (5.0, 3.0)
QUEUE_MARGIN_M = 0.1
CROWD_ZONE_MARGIN_M = 0.6
LANE_CLEARANCE_M = 0.6
GRAVITY = 9.81

# Energy model (teaching assumptions, not measured hardware):
# P = P_idle + (C_rr * m * g * |v| + max(0, m * a * v)) / eta + c_turn * |w|
ENERGY_MODEL = {
    'robot': {'mass_kg': 8.0, 'idle_w': 5.0, 'c_rr': 0.03, 'eta': 0.7, 'turn_w_per_rad_s': 2.0},
    'bus': {'mass_kg': 25.0, 'idle_w': 12.0, 'c_rr': 0.03, 'eta': 0.7, 'turn_w_per_rad_s': 4.0},
}

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
    loop: bool = True           # scripted route: loop (background cars) or stop at the end
    hidden: bool = False        # looping car waiting off the road to re-enter its route
    speed: float = 0.0          # scripted cars: cruise speed
    frozen: bool = False        # observation source frozen (stale stamp)
    in_collision: str = ''
    prev_v: float = 0.0
    power_w: float = 0.0
    energy_j: float = 0.0
    distance_m: float = 0.0


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
        self.barrier = Barrier()
        self.door = Door()
        self.leds = {s: Led(s) for s in hub_map.PARKING_SLOTS}
        self.slot_sensors = {s: Sensor('slot_sensor/' + s, 'slot_sensor')
                             for s in hub_map.PARKING_SLOTS}
        self.camera = Sensor('camera', 'camera')
        self._door_was_open = False
        self.crossing_owner = ''    # from /crossing/state: background cars yield to it
        self.entrance = []          # cars waiting at the parking barrier: [client, since]
        self._passage_until = -math.inf
        self._passing = None
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

    def device_request(self, device, open_):
        """Barrier / door request (from /parking/barrier_request, /bus/door_request)."""
        if device == 'barrier':
            ok, reason = self.barrier.request(open_), self.barrier.reason
        else:
            ok, reason = self.door.request(open_, self.bus_stopped()), self.door.reason
        self._emit(device, 'DEVICE_' + ('OPEN' if open_ else 'CLOSE') +
                   ('' if ok else '_REFUSED'), reason)
        return ok

    def set_slot_states(self, states):
        """LED colours follow the authoritative parking state (/parking/state)."""
        for slot, state in states.items():
            if slot in self.leds:
                self.leds[slot].slot_state = state

    def power_cut(self, reason='ESTOP'):
        """Hardwired stop: propulsion off for bus and robot; no automatic restart."""
        for agent_id in ('bus1', 'robot1'):
            a = self.agents[agent_id]
            if not a.controlled:
                a.route, a.controlled = [], True
            a.v = a.w = 0.0
            a.cmd_time = -math.inf
        self._emit('scenario_runner', 'POWER_CUT', reason)

    def inject(self, ev):
        """Apply one test-only event now (same format as scenario file events)."""
        ev = dict(ev, t=self.t)
        if ev.get('type') in DRIVER_EVENTS:
            self.requests.append(ev)
            return
        handler = getattr(self, '_ev_' + str(ev.get('type')), None)
        if handler is None:
            self._emit('scenario_runner', 'UNKNOWN_EVENT', str(ev.get('type')))
            return
        handler(ev)
        self._emit('scenario_runner', 'INJECT', str(ev['type']))

    # ------------------------------------------------------------------ main loop
    def step(self):
        self._process_events()
        self._arrivals()
        self._expire_pedestrians()
        for a in self.agents.values():
            x0, y0 = a.x, a.y
            if a.controlled:
                self._move_controlled(a)
            elif a.route:
                self._move_scripted(a)
            a.distance_m += math.hypot(a.x - x0, a.y - y0)
            self._integrate_energy(a)
        self._step_devices()
        if self.door.position > 0.0 and abs(self.agents['bus1'].v) > STOPPED_SPEED_MPS:
            self._emit('bus1', 'DOOR_OPEN_MOVING', 'violation: bus moving with door open')
        self._step_entrance()
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
        """
        Pick a free spot in Q for a waiting passenger.

        The robot link R-W runs through the middle of Q (y = 6) and W-Cn leaves it at x = 5,
        so passengers queue in two rows beside the lane, at least LANE_CLEARANCE_M from it,
        never on top of another passenger or a mobile agent. Returns None when Q is full.
        """
        xmin, xmax, ymin, ymax = hub_map.QUEUE_Q
        lanes = [(hub_map.NODES[a], hub_map.NODES[b]) for a, b in hub_map.ROBOT_EDGES]
        lane_y = hub_map.NODES['R'][1]
        for _ in range(500):
            x = self.rng.uniform(xmin + QUEUE_MARGIN_M, xmax - QUEUE_MARGIN_M)
            if self.rng.random() < 0.5:
                y = self.rng.uniform(ymin + QUEUE_MARGIN_M, lane_y - LANE_CLEARANCE_M)
            else:
                y = self.rng.uniform(lane_y + LANE_CLEARANCE_M, ymax - QUEUE_MARGIN_M)
            if any(_point_segment_distance((x, y), a, b) < LANE_CLEARANCE_M - 1e-9
                   for a, b in lanes):
                continue
            if all(math.hypot(x - p.x, y - p.y) > 2 * PEDESTRIAN_RADIUS + 0.05
                   for p in self.peds.values()) and \
                    all(math.hypot(x - a.x, y - a.y) > a.radius + PEDESTRIAN_RADIUS + 0.2
                        for a in self.agents.values()):
                return x, y
        return None

    def queue_peds(self):
        return sorted((p for p in self.peds.values() if p.zone == 'Q'),
                      key=lambda p: p.entered_at)

    def _ev_spawn_queue_tracks(self, ev):
        for _ in range(int(ev.get('count', 1))):
            spot = self._random_queue_point()
            if spot is None:
                self._emit('scenario_runner', 'QUEUE_FULL', 'passenger not spawned')
                return
            self._new_ped('Q', *spot)

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
                'start_fraction': 0.3 + (i + 0.5) / count * 0.6,   # outside crossing C
            })

    def _ev_spawn_random_cars(self, ev):
        """Scalability load: `count` cars looping on random vehicle edges."""
        speed = float(ev.get('speed_mps', 0.35))
        placed = 0
        for i in range(int(ev['count'])):
            car_id = 'load_car%03d' % i
            for _ in range(20):
                a, b = self.rng.choice(hub_map.VEHICLE_EDGES)
                if self.rng.random() < 0.5:
                    a, b = b, a
                self._ev_spawn_car({'id': car_id, 'route': [a, b], 'speed_mps': speed,
                                    'start_fraction': self.rng.uniform(0.1, 0.9)})
                car = self.agents[car_id]
                if not self._collides(car, car.x, car.y):
                    placed += 1
                    break
                del self.agents[car_id]
        self._emit('scenario_runner', 'LOAD_SPAWNED', 'cars=%d/%d' % (placed, int(ev['count'])))

    def _ev_spawn_crowd(self, ev):
        """Scalability load: `count` static pedestrians outside Q and C (zone X)."""
        xmin, xmax, ymin, ymax = hub_map.MAP_BOUNDS
        for _ in range(int(ev['count'])):
            for _ in range(50):
                x = self.rng.uniform(xmin + 0.2, xmax - 0.2)
                y = self.rng.uniform(ymin + 0.2, ymax - 0.2)
                if _near_rect(x, y, hub_map.QUEUE_Q, CROWD_ZONE_MARGIN_M) or \
                        _near_rect(x, y, hub_map.CROSSING_C, CROWD_ZONE_MARGIN_M):
                    continue   # well clear of Q and C, so sensor noise never moves them in
                edges = hub_map.VEHICLE_EDGES + hub_map.PARKING_EDGES + hub_map.ROBOT_EDGES
                if any(_point_segment_distance((x, y), hub_map.NODES[a], hub_map.NODES[b]) < 0.7
                       for a, b in edges):
                    continue   # keep roads and robot links free
                if all(math.hypot(x - p.x, y - p.y) > 2 * PEDESTRIAN_RADIUS
                       for p in self.peds.values()):
                    self._new_ped('X', x, y)
                    break

    def _ev_car_at_entrance(self, ev):
        """Queue a car at the parking barrier; the entrance sensor reports its id."""
        self.entrance.append([ev['client'], self.t])

    def entrance_vehicle(self):
        """Vehicle id currently seen by the entrance sensor ('' when the lane is empty)."""
        return self.entrance[0][0] if self.entrance else ''

    def _step_entrance(self):
        if self._passing is not None and self.t >= self._passage_until:
            client = self._passing
            slot = self.reservations.get(client) or next(
                (s for s, occ in sorted(self.slots.items()) if not occ), '')
            if slot:
                self.slots[slot] = True
            self._emit(client, 'CAR_PARKED', slot or 'no free slot')
            self._passing = None
            self.barrier.passage_occupied = False
        if not self.entrance or self._passing is not None:
            return
        client, since = self.entrance[0]
        if self.barrier.is_open:
            self.entrance.pop(0)
            self._passing, self._passage_until = client, self.t + 2.0
            self.barrier.passage_occupied = True          # the barrier may not close now
            self._emit(client, 'CAR_ENTERING', 'barrier open')
        elif self.t - since > 15.0:
            self.entrance.pop(0)
            self._emit(client, 'CAR_LEFT', 'barrier stayed closed')

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
        if self.estop:
            self.power_cut('ESTOP')

    def _ev_agent_route(self, ev):
        """Scripted motion of bus1 / robot1 along graph nodes (simulator test mode)."""
        a = self.agents[ev['agent']]
        route = list(ev['route'])
        start = hub_map.NODES[route[0]]
        if math.hypot(a.x - start[0], a.y - start[1]) > 0.3:
            route.insert(0, min(hub_map.NODES, key=lambda n: math.dist((a.x, a.y),
                                                                       hub_map.NODES[n])))
        a.route, a.route_index, a.loop = route, 1 if len(route) > 1 else 0, False
        a.speed = float(ev.get('speed_mps', 0.25 if a.kind == 'robot' else 0.35))
        a.controlled = False

    def _ev_barrier(self, ev):
        self.device_request('barrier', bool(ev['open']))

    def _ev_door(self, ev):
        self.device_request('door', bool(ev['open']))

    # ------------------------------------------------------------------ dynamics
    def _arrivals(self):
        if self.arrival_rate_per_min > 0.0 and \
                self.rng.random() < self.arrival_rate_per_min / 60.0 * STEP_SEC:
            spot = self._random_queue_point()
            if spot is not None:
                self._new_ped('Q', *spot)

    def _expire_pedestrians(self):
        for pid in [k for k, p in self.peds.items() if p.leave_at <= self.t]:
            del self.peds[pid]

    def _obstructions(self, agent):
        """Yield (id, x, y, radius) of everything the agent may collide with."""
        for o in self.agents.values():
            if o is not agent and not o.hidden:
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
            if a.v != 0.0 or a.w != 0.0:
                # Actuator-side watchdog: final command expired (e.g. supervisor crash).
                self._emit(a.agent_id, 'CMD_TIMEOUT', 'final command expired, motors zero')
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
        if a.hidden:
            sx, sy = hub_map.NODES[a.route[0]]
            if not self._collides(a, sx, sy):
                a.x, a.y, a.route_index, a.hidden = sx, sy, 1, False
            return
        tx, ty = hub_map.NODES[a.route[a.route_index]]
        dx, dy = tx - a.x, ty - a.y
        remaining = math.hypot(dx, dy)
        step = min(a.speed * STEP_SEC, remaining)
        nx = a.x + step * dx / remaining if remaining > 0 else a.x
        ny = a.y + step * dy / remaining if remaining > 0 else a.y
        # Background cars obey the crossing: no entry into C while someone owns it.
        if a.kind == 'car' and self.crossing_owner and \
                _footprint_in_c(nx, ny, a.radius) and not _footprint_in_c(a.x, a.y, a.radius):
            a.v = 0.0
            return
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
            if not a.loop:
                # Scripted bus / robot route finished: stop and hand back to ROS control.
                self._emit(a.agent_id, 'ROUTE_DONE', '-'.join(a.route))
                a.route, a.v, a.controlled = [], 0.0, a.kind in ('bus', 'robot')
                a.cmd_time = -math.inf
                return
            # Loop: re-enter at the route start to keep a steady flow; while the start is
            # occupied the car leaves the road instead of blocking the end node.
            sx, sy = hub_map.NODES[a.route[0]]
            if self._collides(a, sx, sy):
                a.v, a.hidden = 0.0, True
                return
            a.x, a.y, a.route_index = sx, sy, 1

    def _integrate_energy(self, a):
        model = ENERGY_MODEL.get(a.kind)
        if model is None:
            return
        v = abs(a.v)
        accel = (v - abs(a.prev_v)) / STEP_SEC
        traction = model['c_rr'] * model['mass_kg'] * GRAVITY * v + \
            max(0.0, model['mass_kg'] * accel * v)
        a.power_w = model['idle_w'] + traction / model['eta'] + \
            model['turn_w_per_rad_s'] * abs(a.w)
        a.energy_j += a.power_w * STEP_SEC
        a.prev_v = a.v

    def energy_report(self):
        """[(agent_id, power_w, energy_j, distance_m)] for agents with an energy model."""
        return [(a.agent_id, a.power_w, a.energy_j, a.distance_m)
                for a in sorted(self.agents.values(), key=lambda q: q.agent_id)
                if a.kind in ENERGY_MODEL]

    def _step_devices(self):
        self.barrier.step(STEP_SEC)
        self.door.step(STEP_SEC)
        for led in self.leds.values():
            led.step(STEP_SEC, self.t)
        for slot, sensor in self.slot_sensors.items():
            sensor.online = self.slot_connected[slot]
            sensor.step(STEP_SEC)
        self.camera.online = not self.camera_frozen()
        self.camera.step(STEP_SEC)
        if self.door.position >= 1.0 and not self._door_was_open:
            # First passenger boards as soon as the door is open, then one every 2 s.
            self._boarded_this_stop = 0
            self._next_boarding_t = self.t
        self._door_was_open = self.door.position >= 1.0

    def devices(self):
        return [self.barrier, self.door, *self.leds.values(), *self.slot_sensors.values(),
                self.camera]

    def bus_stopped(self):
        return abs(self.agents['bus1'].v) < STOPPED_SPEED_MPS

    def bus_docked(self):
        bus = self.agents['bus1']
        bx, by = hub_map.NODES[hub_map.BUS_BAY]
        return math.hypot(bus.x - bx, bus.y - by) <= DOCK_TOLERANCE_M and \
            abs(bus.v) < STOPPED_SPEED_MPS

    def _boarding(self):
        if not (self.door.position >= 1.0 and self.bus_docked()):
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
        """Edge the agent travels on; '' when it stands at a graph node or off the graph."""
        if agent.kind != 'obstacle' and any(
                math.hypot(agent.x - nx, agent.y - ny) <= NODE_RADIUS_M
                for nx, ny in hub_map.NODES.values()):
            return ''
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
            if a.hidden:
                continue
            noise = 0.0 if a.kind == 'obstacle' else noise_m
            out.append((a, a.x + self.rng.gauss(0.0, noise), a.y + self.rng.gauss(0.0, noise),
                        a.yaw, abs(a.v), self.edge_of(a)))
        return out

    def observed_slots(self):
        return {s: occ for s, occ in self.slots.items() if self.slot_connected[s]}


def _near_rect(x, y, rect, margin):
    xmin, xmax, ymin, ymax = rect
    return xmin - margin <= x <= xmax + margin and ymin - margin <= y <= ymax + margin


def _footprint_in_c(x, y, radius):
    xmin, xmax, ymin, ymax = hub_map.CROSSING_C
    return xmin - radius <= x <= xmax + radius and ymin - radius <= y <= ymax + radius


def apply_variant(scenario, variant):
    """
    Return the scenario with a test variant applied.

    A variant lists extra events; an item {type: remove, event_type: X} drops the base
    events of type X. Unknown variants raise KeyError.
    """
    if not variant:
        return scenario
    variants = scenario.get('variants') or {}
    if variant not in variants:
        raise KeyError('unknown variant %r (have: %s)' % (
            variant, ', '.join(sorted(variants)) or 'none'))
    extra = list(variants[variant] or [])
    dropped = {e['event_type'] for e in extra if e.get('type') == 'remove'}
    events = [e for e in scenario.get('events', []) if e['type'] not in dropped]
    events += [e for e in extra if e.get('type') != 'remove']
    return dict(scenario, events=events)


def _point_segment_distance(p, a, b):
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))
