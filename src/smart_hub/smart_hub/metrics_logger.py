# Author: Mikhail Li (12236295)
"""
metrics_logger.

Records every run (assignment 21.2): run ID, scenario, seed, parameters and provenance
(commit, file hashes, ROS distribution, RMW, machine, evaluator revision); all /hub/events;
latency samples (event `latency_ms=` fields, e.g. stop latch, crossing grant, route
compute, service round trips); data age on key topics (receipt time - observation stamp,
i.e. end-to-end sensor latency) and topic rates; energy per device; collisions, safety
interventions and mission outcomes.

Crossing C is evaluated from private ground truth with no grace interval
(metrics.CrossingMonitor): strict occupancy, unauthorized entry, token overlap, pedestrian
response latency and contact are separate measures, and every change of the occupant set
is kept in crossing_occupancy.csv. Passenger waits cover everyone who entered the queue
(metrics.QueueLedger): boarded, abandoned and still waiting at run end (right-censored).
A REROUTE or NO_ROUTE event starts a stop watch on the agent's ground-truth speed
(metrics.StopMonitor): ROUTE_STOP_MEASURED is the time to measured standstill, and motion
after a NO_ROUTE stop is reported as MOTION_WITHOUT_ROUTE.

Clock domains: event times and data ages use ROS header stamps (one host, one clock);
`latency_ms=` fields are monotonic intervals measured inside the emitting node; ground
truth is sampled at the simulator's truth rate (10 Hz), which bounds the resolution of
occupancy seconds and of the pedestrian response latency to 100 ms.

Writes <output_dir>/<run_id>/{events,latency,energy,crossing_occupancy}.csv and
summary.json, and publishes a 1 Hz summary on /hub/metrics for the dashboard.
"""
from collections import Counter, deque
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
import platform
import re
import subprocess
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters, ListParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.time import Time
from sensor_msgs.msg import Image
from smart_hub.freshness import Retained
from smart_hub.hub_events import ack_reset
from smart_hub.metrics import (
    CrossingMonitor, parse_latency_ms, parse_wait_sec, QueueLedger,
    relaxed_crossing_conflict, Series, StopMonitor,
)
from smart_hub.params import check_parameters
from smart_hub.qos import R10, R100, R1T, SENSOR
from smart_hub_interfaces.msg import (
    CrossingState, DemandPrediction, EdgeStateArray, HubEvent, ParkingState, QueueState,
    SafetyState, VehicleStateArray,
)

AGE_TOPICS = [
    ('/hub/queue', QueueState, R10),
    ('/hub/demand', DemandPrediction, R10),
    ('/hub/vehicles', VehicleStateArray, R10),
    ('/traffic/edges', EdgeStateArray, R10),
    ('/crossing/state', CrossingState, R1T),
    ('/parking/state', ParkingState, R1T),
    ('/hub/safety', SafetyState, R1T),
    ('/robot/odom', Odometry, R10),
    ('/sim/pedestrians', PoseArray, R10),
    ('/camera/image_raw', Image, SENSOR),
]
RATE_WINDOW_SEC = 5.0
CROSSING_STALE_SEC = 1.0
RELAXED_REACTION_SEC = 1.0      # midterm diagnostic only, not used by any reported measure
EVALUATOR = 'metrics_logger r2: strict crossing occupancy, censored waits, provenance'
HASHED_FILES = ('config/hub.yaml', 'config/map.yaml', 'models/demand_ridge.json')


def _git(args, cwd):
    try:
        return subprocess.run(['git'] + args, capture_output=True, text=True, timeout=2,
                              cwd=cwd).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ''


def source_revision():
    """
    Return (full commit hash, dirty flag) of the code under test.

    SMART_HUB_COMMIT (set by the test matrix, or by hand for an installed copy without a
    git checkout) wins; otherwise the working directory and then the source tree behind a
    symlink install are asked.
    """
    commit = os.environ.get('SMART_HUB_COMMIT', '')
    if commit:
        return commit, os.environ.get('SMART_HUB_DIRTY', '') == '1'
    for cwd in (os.getcwd(), os.path.dirname(os.path.realpath(__file__))):
        commit = _git(['rev-parse', 'HEAD'], cwd)
        if commit:
            return commit, bool(_git(['status', '--porcelain', '--untracked-files=no'], cwd))
    return 'unknown', False


def file_sha256(path):
    try:
        with open(path, 'rb') as f:
            return hashlib.sha256(f.read()).hexdigest()
    except OSError:
        return 'unknown'


def share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('smart_hub')
    except Exception:
        return ''


def _proc_field(path, key):
    try:
        with open(path) as f:
            for line in f:
                if line.startswith(key):
                    return line.split(':', 1)[1].strip()
    except OSError:
        pass
    return ''


def run_metadata():
    """Provenance of the run record (21.2): code, configuration, middleware, machine."""
    commit, dirty = source_revision()
    share = share_dir()
    hashes = {rel: file_sha256(os.path.join(share, rel)) for rel in HASHED_FILES}
    try:
        rmw = rclpy.get_rmw_implementation_identifier()
    except Exception:
        rmw = 'unknown'
    ram_kb = _proc_field('/proc/meminfo', 'MemTotal').split(' ')[0]
    return {'commit': commit, 'commit_dirty': dirty,
            'config_sha256': hashes['config/hub.yaml'], 'files_sha256': hashes,
            'evaluator': EVALUATOR,
            'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'), 'rmw': rmw,
            'machine': '%s %s, %d CPU' % (platform.system(), platform.machine(),
                                          os.cpu_count() or 0),
            'os': platform.platform(),
            'cpu_model': _proc_field('/proc/cpuinfo', 'model name') or 'unknown',
            'ram_gb': round(int(ram_kb) / 1048576.0, 1) if ram_kb.isdigit() else None,
            'python': platform.python_version()}


class MetricsLogger(Node):

    def __init__(self):
        super().__init__('metrics_logger')
        self.declare_parameter('output_dir', 'evidence/runs')
        self.declare_parameter('write_files', True)
        check_parameters(self)

        self.metrics_pub = self.create_publisher(DiagnosticArray, '/hub/metrics', R10)

        self.events_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.create_subscription(DiagnosticArray, '/sim/energy', self.on_energy, R10)
        self.create_subscription(VehicleStateArray, '/sim/truth/vehicles', self.on_truth_v, R10)
        self.create_subscription(PoseArray, '/sim/truth/pedestrians', self.on_truth_p, R10)
        for topic, msg_type, qos in AGE_TOPICS:
            self.create_subscription(msg_type, topic,
                                     lambda m, t=topic: self.on_stamped(t, m), qos)
        self.metadata = run_metadata()
        self.param_clients = {}
        self.start_run('', '', '')
        self.create_timer(1.0, self.publish)
        self.create_timer(5.0, self.write_summary)

    # ------------------------------------------------------------------ run bookkeeping
    def start_run(self, run_id, scenario_id, seed):
        self.run_id, self.scenario_id, self.seed = run_id, scenario_id, seed
        self.started = time.monotonic()
        self.started_ros = self.get_clock().now().nanoseconds / 1e9
        self.latency = {}
        self.age = {}
        self.receipts = {}
        self.counts = Counter()
        self.energy = {}
        self.started_utc = datetime.now(timezone.utc).isoformat(timespec='seconds')
        self.scenario_sha256 = file_sha256(os.path.join(
            share_dir(), 'scenarios', '%s.yaml' % scenario_id)) if scenario_id else ''
        self.passengers = QueueLedger()
        self.mission_time = Series()
        self.goal_started = {}
        self.truth_agents, self.truth_peds = [], []
        self.truth_stamp = {}
        self.crossing = CrossingMonitor()
        self.route_stops = StopMonitor()
        self.crossing_state = Retained(CROSSING_STALE_SEC)
        self.relaxed_conflicts = 0
        self.in_conflict = False
        self.conflict_since = None
        self.files = {}
        if run_id and self.get_parameter('write_files').value:
            run_dir = os.path.join(self.get_parameter('output_dir').value, run_id)
            os.makedirs(run_dir, exist_ok=True)
            self.run_dir = run_dir
            for name, header in (
                    ('events', ['t_sec', 'entity_id', 'event', 'reason', 'correlation_id']),
                    ('latency', ['t_sec', 'metric', 'value_ms']),
                    ('crossing_occupancy', ['t_sec', 'agents_in_c', 'pedestrians_in_c',
                                            'owner', 'token', 'occupancy_classes']),
                    ('energy', ['t_sec', 'device', 'power_w', 'energy_j'])):
                f = open(os.path.join(run_dir, name + '.csv'), 'w', newline='')
                writer = csv.writer(f)
                writer.writerow(header)
                self.files[name] = (f, writer)
        else:
            self.run_dir = ''

    def dump_parameters(self):
        """Record every node's final parameter values with the run (assignment 17)."""
        if not self.run_dir:
            return
        run_dir, collected = self.run_dir, {}
        names = [n for n in self.get_node_names() if n not in ('metrics_logger',)]
        pending = set(names)

        def finish(name):
            pending.discard(name)
            if not pending:
                with open(os.path.join(run_dir, 'params.json'), 'w') as f:
                    json.dump(collected, f, indent=2, sort_keys=True)

        for name in names:
            lister = self.param_client(ListParameters, name, 'list_parameters')
            if not lister.service_is_ready():
                finish(name)
                continue

            def listed(future, name=name):
                getter = self.param_client(GetParameters, name, 'get_parameters')
                keys = list(future.result().result.names)
                get = getter.call_async(GetParameters.Request(names=keys))
                get.add_done_callback(lambda f, keys=keys, name=name: (
                    collected.__setitem__(name, {k: parameter_value_to_python(v)
                                                 for k, v in zip(keys, f.result().values)}),
                    finish(name)))

            lister.call_async(ListParameters.Request()).add_done_callback(listed)
        own = self.get_parameters(self.list_parameters([], 0).names)
        collected['metrics_logger'] = {p.name: p.value for p in own}

    def param_client(self, srv, node_name, suffix):
        key = (node_name, suffix)
        if key not in self.param_clients:
            self.param_clients[key] = self.create_client(srv, '/%s/%s' % (node_name, suffix))
        return self.param_clients[key]

    def t(self, stamp=None):
        """Run time of an event: its header stamp when set, else the receipt time."""
        if stamp is not None and (stamp.sec or stamp.nanosec):
            # Events can arrive late while the ROS graph is still connecting; the stamp
            # keeps the record in the order the events actually happened.
            return Time.from_msg(stamp).nanoseconds / 1e9 - self.started_ros
        return time.monotonic() - self.started

    def row(self, name, values):
        if name in self.files:
            f, writer = self.files[name]
            writer.writerow(values)
            f.flush()

    def close_files(self):
        for f, _ in self.files.values():
            f.close()
        self.files = {}

    # ------------------------------------------------------------------ inputs
    def on_event(self, msg):
        # A run starts with RUN_START; any later simulator event with a new run_id also
        # starts it, in case RUN_START was published before this subscription matched.
        if msg.entity_id == 'scenario_runner' and msg.run_id and msg.run_id != self.run_id:
            self.write_summary()
            self.close_files()
            seed = re.search(r'-s(\d+)-', msg.run_id)
            self.start_run(msg.run_id, msg.scenario_id, seed.group(1) if seed else '')
            self.dump_parameters()
        self.counts[msg.event] += 1
        self.row('events', ['%.3f' % self.t(msg.header.stamp), msg.entity_id, msg.event,
                            msg.reason,
                            msg.correlation_id])
        latency = parse_latency_ms(msg.reason)
        if latency is not None:
            key = msg.event
            self.latency.setdefault(key, Series()).add(latency)
            self.row('latency', ['%.3f' % self.t(msg.header.stamp), key, '%.3f' % latency])
        t = self.t(msg.header.stamp)
        if msg.event == 'QUEUE_ENTERED':
            self.passengers.enter(msg.entity_id, t)
        elif msg.event == 'BOARDED':
            self.passengers.board(msg.entity_id, t, parse_wait_sec(msg.reason))
        elif msg.event == 'LEFT_QUEUE':
            self.passengers.abandon(msg.entity_id, t)
        elif msg.event == 'QUEUE_BALKED':
            self.passengers.balked += 1
        elif msg.event in ('CROSSING_GRANTED', 'CROSSING_RELEASED', 'CROSSING_FREE'):
            self.crossing.on_event(msg.event, msg.correlation_id)
        elif msg.event in ('REROUTE', 'NO_ROUTE'):
            self.route_stops.demand(msg.entity_id, msg.event, t)
        if msg.event == 'GOAL_ACCEPTED':
            self.route_stops.release(msg.entity_id)
            self.goal_started[msg.correlation_id] = time.monotonic()
        elif msg.event in ('GOAL_SUCCEED', 'GOAL_ABORT', 'GOAL_CANCELED'):
            start = self.goal_started.pop(msg.correlation_id, None)
            if start is not None and msg.event == 'GOAL_SUCCEED':
                self.mission_time.add(time.monotonic() - start)
        ack_reset(self, self.events_pub, msg)

    def on_stamped(self, topic, msg):
        receipt = time.monotonic()
        stamp = msg.header.stamp
        if topic == '/crossing/state':
            self.crossing_state.put(msg, receipt)
        if stamp.sec or stamp.nanosec:
            age_ms = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e6
            self.age.setdefault(topic, Series(keep=2000)).add(age_ms)
        window = self.receipts.setdefault(topic, deque())
        window.append(receipt)
        while window and receipt - window[0] > RATE_WINDOW_SEC:
            window.popleft()

    def on_energy(self, msg):
        for s in msg.status:
            values = {kv.key: kv.value for kv in s.values}
            self.energy[s.name[len('energy/'):]] = values
            self.row('energy', ['%.3f' % self.t(), s.name, values.get('power_w', ''),
                                values.get('energy_j', '')])

    def on_truth_v(self, msg):
        self.truth_agents = [(v.vehicle_id, v.kind, v.pose.x, v.pose.y, v.speed_mps)
                             for v in msg.vehicles]
        self.on_truth('vehicles', msg.header.stamp)

    def on_truth_p(self, msg):
        self.truth_peds = [(p.position.x, p.position.y) for p in msg.poses]
        self.on_truth('pedestrians', msg.header.stamp)

    def on_truth(self, source, stamp):
        # Vehicles and pedestrians of one simulator step carry the same stamp; the
        # crossing is sampled once both halves of that step have arrived.
        self.truth_stamp[source] = (stamp.sec, stamp.nanosec)
        if self.truth_stamp.get('vehicles') != self.truth_stamp.get('pedestrians'):
            return
        now = self.get_clock().now().nanoseconds / 1e9
        state = self.crossing_state.get(now, time.monotonic())
        t = self.t(stamp)
        row, found = self.crossing.sample(
            t, self.truth_agents, self.truth_peds,
            None if state is None else state.owner_id, '' if state is None else state.token)
        if row:
            self.row('crossing_occupancy', row)
        for event, entity, reason in found + self.route_stops.sample(t, self.truth_agents):
            self.row('events', ['%.3f' % t, entity, event, reason, 'ground_truth'])
            latency = parse_latency_ms(reason)
            if latency is not None:
                self.row('latency', ['%.3f' % t, event, '%.3f' % latency])
        self.check_relaxed()

    def check_relaxed(self):
        # Midterm definition, kept as a diagnostic for comparison with the earlier record:
        # car-only occupancy ignored and a pedestrian next to a robot / bus counted only if
        # the agent is still moving RELAXED_REACTION_SEC later. Not a reported measure.
        now = time.monotonic()
        two_agents = relaxed_crossing_conflict(self.truth_agents, [])
        with_ped = relaxed_crossing_conflict(self.truth_agents, self.truth_peds)
        if with_ped and not two_agents:
            self.conflict_since = self.conflict_since or now
            with_ped = now - self.conflict_since > RELAXED_REACTION_SEC
        else:
            self.conflict_since = None
        conflict = two_agents or with_ped
        if conflict and not self.in_conflict:
            self.relaxed_conflicts += 1
        self.in_conflict = conflict

    # ------------------------------------------------------------------ outputs
    def rate(self, topic):
        window = self.receipts.get(topic)
        if not window or len(window) < 2:
            return 0.0
        span = window[-1] - window[0]
        return (len(window) - 1) / span if span > 0 else 0.0

    def summary(self):
        c = self.counts
        crossing = self.crossing.summary()
        crossing.update(contacts=c['COLLISION'],
                        relaxed_conflicts_diagnostic=self.relaxed_conflicts)
        queue = self.passengers.summary(
            self.get_clock().now().nanoseconds / 1e9 - self.started_ros)
        return {
            'run_id': self.run_id, 'scenario_id': self.scenario_id, 'seed': self.seed,
            'elapsed_sec': round(self.t(), 1), 'started_utc': self.started_utc,
            'written_utc': datetime.now(timezone.utc).isoformat(timespec='seconds'),
            'scenario_sha256': self.scenario_sha256, **self.metadata,
            'latency_ms': {k: s.summary() for k, s in sorted(self.latency.items())},
            'topic_age_ms': {k: s.summary() for k, s in sorted(self.age.items())},
            'topic_rate_hz': {t: round(self.rate(t), 2) for t, _, _ in AGE_TOPICS},
            'energy': self.energy,
            'counts': {
                'collisions': c['COLLISION'],
                'crossing_strict_occupancy': crossing['strict_occupancy']['episodes'],
                'crossing_unauthorized_entries': sum(
                    crossing['unauthorized_entries'].values()),
                'crossing_token_overlaps': crossing['token_overlaps'],
                'safety_interventions': c['SAFETY_INHIBIT'] + c['STOP_LATCHED'],
                'goals_succeeded': c['GOAL_SUCCEED'],
                'goals_aborted': c['GOAL_ABORT'],
                'goals_canceled': c['GOAL_CANCELED'],
                'goals_rejected': c['GOAL_REJECTED'],
                'reroutes': c['REROUTE'],
                'crossing_grants': c['CROSSING_GRANTED'],
                'boarded': c['BOARDED'],
            },
            'crossing': crossing,
            'route_stops': self.route_stops.summary(),
            'queue': queue,
            'queue_wait_sec': queue['wait_boarded_sec'],     # boarded passengers only
            'mission_time_sec': self.mission_time.summary(),
        }

    def write_summary(self):
        if self.run_dir:
            path = os.path.join(self.run_dir, 'summary.json')
            with open(path + '.tmp', 'w') as f:          # atomic: readers never see half
                json.dump(self.summary(), f, indent=2)
            os.replace(path + '.tmp', path)

    def publish(self):
        data = self.summary()
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        def status(name, values):
            s = DiagnosticStatus()
            s.name = name
            s.values = [KeyValue(key=str(k), value=str(round(v, 3) if isinstance(v, float)
                                                       else v)) for k, v in values.items()]
            msg.status.append(s)

        status('run', {k: data[k] for k in ('run_id', 'scenario_id', 'seed', 'elapsed_sec',
                                            'commit', 'config_sha256', 'machine',
                                            'ros_distro', 'rmw')})
        status('counts', data['counts'])
        crossing, queue = data['crossing'], data['queue']
        status('crossing', dict(
            {'strict_' + k: v['episodes']
             for k, v in crossing['strict_occupancy']['by_class'].items()},
            strict_seconds=crossing['strict_occupancy']['seconds'],
            **{'entry_' + k: v for k, v in crossing['unauthorized_entries'].items()},
            pedestrian_response_p95_ms=crossing['pedestrian_response_ms'].get('p95', ''),
            pedestrian_response_unresolved=crossing['pedestrian_response_unresolved']))
        status('queue', {k: ('' if queue[k] is None else queue[k]) for k in (
            'arrived', 'boarded', 'abandoned', 'waiting_at_end', 'balked',
            'completion_rate', 'clearance_rate')})
        status('queue_wait_censored_sec', queue['wait_censored_sec'])
        status('queue_wait_all_sec', queue['wait_all_lower_bound_sec'])
        for k, v in data['latency_ms'].items():
            status('latency/' + k, v)
        for k, v in data['topic_age_ms'].items():
            status('topic_age/' + k, v)
        status('topic_rate', data['topic_rate_hz'])
        for k, v in data['energy'].items():
            status('energy/' + k, v)
        status('queue_wait_sec', data['queue_wait_sec'])
        status('mission_time_sec', data['mission_time_sec'])
        self.metrics_pub.publish(msg)

    def destroy_node(self):
        self.write_summary()
        self.close_files()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
