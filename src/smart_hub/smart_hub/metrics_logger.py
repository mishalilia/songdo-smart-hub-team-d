"""
metrics_logger — owner M5.

Records every run (assignment 21.2): run ID, scenario, seed, parameters; all /hub/events;
latency samples (event `latency_ms=` fields, e.g. stop latch, crossing grant, route
compute, service round trips); data age on key topics (receipt time - observation stamp,
i.e. end-to-end sensor latency) and topic rates; energy per device; collisions, crossing
conflicts from private ground truth, safety interventions and mission outcomes.
Writes <output_dir>/<run_id>/{events,latency,energy}.csv and summary.json, and publishes a
1 Hz summary on /hub/metrics for the dashboard.
"""
from collections import Counter, deque
import csv
import hashlib
import json
import os
import platform
import subprocess
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from smart_hub.metrics import (
    crossing_conflict, parse_latency_ms, parse_wait_sec, Series,
)
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


def run_metadata():
    """Commit hash, configuration hash and machine for the run record (21.2)."""
    try:
        commit = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], capture_output=True,
                                text=True, timeout=2).stdout.strip() or 'unknown'
    except (OSError, subprocess.SubprocessError):
        commit = 'unknown'
    config_hash = 'unknown'
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('smart_hub'), 'config', 'hub.yaml')
        with open(path, 'rb') as f:
            config_hash = hashlib.sha256(f.read()).hexdigest()[:12]
    except Exception:
        pass
    return {'commit': commit, 'config_sha256': config_hash,
            'machine': '%s %s, %d CPU' % (platform.system(), platform.machine(),
                                          os.cpu_count() or 0),
            'python': platform.python_version()}


class MetricsLogger(Node):

    def __init__(self):
        super().__init__('metrics_logger')
        self.declare_parameter('output_dir', 'evidence/runs')
        self.declare_parameter('write_files', True)

        self.metrics_pub = self.create_publisher(DiagnosticArray, '/hub/metrics', R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.create_subscription(DiagnosticArray, '/sim/energy', self.on_energy, R10)
        self.create_subscription(VehicleStateArray, '/sim/truth/vehicles', self.on_truth_v, R10)
        self.create_subscription(PoseArray, '/sim/truth/pedestrians', self.on_truth_p, R10)
        for topic, msg_type, qos in AGE_TOPICS:
            self.create_subscription(msg_type, topic,
                                     lambda m, t=topic: self.on_stamped(t, m), qos)
        self.metadata = run_metadata()
        self.start_run('', '', '')
        self.create_timer(1.0, self.publish)
        self.create_timer(5.0, self.write_summary)

    # ------------------------------------------------------------------ run bookkeeping
    def start_run(self, run_id, scenario_id, seed):
        self.run_id, self.scenario_id, self.seed = run_id, scenario_id, seed
        self.started = time.monotonic()
        self.latency = {}
        self.age = {}
        self.receipts = {}
        self.counts = Counter()
        self.energy = {}
        self.waits = Series()
        self.mission_time = Series()
        self.goal_started = {}
        self.truth_agents, self.truth_peds = [], []
        self.in_conflict = False
        self.files = {}
        if run_id and self.get_parameter('write_files').value:
            run_dir = os.path.join(self.get_parameter('output_dir').value, run_id)
            os.makedirs(run_dir, exist_ok=True)
            self.run_dir = run_dir
            for name, header in (
                    ('events', ['t_sec', 'entity_id', 'event', 'reason', 'correlation_id']),
                    ('latency', ['t_sec', 'metric', 'value_ms']),
                    ('energy', ['t_sec', 'device', 'power_w', 'energy_j'])):
                f = open(os.path.join(run_dir, name + '.csv'), 'w', newline='')
                writer = csv.writer(f)
                writer.writerow(header)
                self.files[name] = (f, writer)
        else:
            self.run_dir = ''

    def t(self):
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
        if msg.event == 'RUN_START':
            self.write_summary()
            self.close_files()
            seed = msg.reason.split('=')[-1] if msg.reason.startswith('seed=') else ''
            self.start_run(msg.run_id, msg.scenario_id, seed)
        self.counts[msg.event] += 1
        self.row('events', ['%.3f' % self.t(), msg.entity_id, msg.event, msg.reason,
                            msg.correlation_id])
        latency = parse_latency_ms(msg.reason)
        if latency is not None:
            key = msg.event
            self.latency.setdefault(key, Series()).add(latency)
            self.row('latency', ['%.3f' % self.t(), key, '%.3f' % latency])
        if msg.event == 'BOARDED':
            wait = parse_wait_sec(msg.reason)
            if wait is not None:
                self.waits.add(wait)
        if msg.event == 'GOAL_ACCEPTED':
            self.goal_started[msg.correlation_id] = time.monotonic()
        elif msg.event in ('GOAL_SUCCEED', 'GOAL_ABORT', 'GOAL_CANCELED'):
            start = self.goal_started.pop(msg.correlation_id, None)
            if start is not None and msg.event == 'GOAL_SUCCEED':
                self.mission_time.add(time.monotonic() - start)

    def on_stamped(self, topic, msg):
        receipt = time.monotonic()
        stamp = msg.header.stamp
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
        self.check_conflict()

    def on_truth_p(self, msg):
        self.truth_peds = [(p.position.x, p.position.y) for p in msg.poses]

    def check_conflict(self):
        conflict = crossing_conflict(self.truth_agents, self.truth_peds)
        if conflict and not self.in_conflict:
            self.counts['CROSSING_CONFLICT'] += 1
            self.row('events', ['%.3f' % self.t(), 'metrics_logger', 'CROSSING_CONFLICT',
                                'ground truth', ''])
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
        return {
            'run_id': self.run_id, 'scenario_id': self.scenario_id, 'seed': self.seed,
            'elapsed_sec': round(self.t(), 1), **self.metadata,
            'latency_ms': {k: s.summary() for k, s in sorted(self.latency.items())},
            'topic_age_ms': {k: s.summary() for k, s in sorted(self.age.items())},
            'topic_rate_hz': {t: round(self.rate(t), 2) for t, _, _ in AGE_TOPICS},
            'energy': self.energy,
            'counts': {
                'collisions': c['COLLISION'],
                'crossing_conflicts': c['CROSSING_CONFLICT'],
                'safety_interventions': c['SAFETY_INHIBIT'] + c['STOP_LATCHED'],
                'goals_succeeded': c['GOAL_SUCCEED'],
                'goals_aborted': c['GOAL_ABORT'],
                'goals_canceled': c['GOAL_CANCELED'],
                'goals_rejected': c['GOAL_REJECTED'],
                'reroutes': c['REROUTE'],
                'crossing_grants': c['CROSSING_GRANTED'],
                'boarded': c['BOARDED'],
            },
            'queue_wait_sec': self.waits.summary(),
            'mission_time_sec': self.mission_time.summary(),
        }

    def write_summary(self):
        if self.run_dir:
            with open(os.path.join(self.run_dir, 'summary.json'), 'w') as f:
                json.dump(self.summary(), f, indent=2)

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
                                            'commit', 'config_sha256', 'machine')})
        status('counts', data['counts'])
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
