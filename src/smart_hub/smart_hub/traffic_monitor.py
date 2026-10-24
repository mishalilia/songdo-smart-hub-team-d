# Author: Vladislav Li (12250245)
"""
traffic_monitor (scenario S5).

Per-edge speed, density rho = min(1, n/K), congestion and blockage from /hub/vehicles,
5 s sliding window (traffic_logic.TrafficEstimator). Publishes /traffic/edges at 2 Hz and
NORMAL / CONGESTED / BLOCKED / UNKNOWN transitions on /hub/events. Stale edge =>
valid=false, never "uncongested". See assignment 12.
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from smart_hub.hub_events import ack_reset
from smart_hub.params import check_parameters
from smart_hub.qos import R10, R100
from smart_hub.traffic_logic import TrafficEstimator
from smart_hub_interfaces.msg import EdgeState, EdgeStateArray, HubEvent, VehicleStateArray

FREE_FLOW_MPS = 0.35
SLOW_PUBLICATIONS = 3


class TrafficMonitor(Node):

    def __init__(self):
        super().__init__('traffic_monitor')
        self.declare_parameter('congestion_density_threshold', 0.70)
        self.declare_parameter('slow_speed_ratio', 0.40)
        self.declare_parameter('window_sec', 5.0)
        self.declare_parameter('headway_m', 1.0)
        check_parameters(self)

        self.estimator = TrafficEstimator(
            window_sec=self.get_parameter('window_sec').value,
            headway_m=self.get_parameter('headway_m').value)
        self.labels = {}
        self.slow_count = {}
        self.run_id = ''

        self.edges_pub = self.create_publisher(EdgeStateArray, '/traffic/edges', R10)
        self.events_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.create_subscription(VehicleStateArray, '/hub/vehicles', self.on_vehicles, R10)
        self.create_subscription(HubEvent, '/hub/events', self.on_event, R100)
        self.create_timer(0.1, self.tick)
        self.ticks = 0

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_event(self, msg):
        if msg.event in ('RESET', 'RUN_START'):
            self.estimator.reset()
            self.labels.clear()
            self.run_id = msg.correlation_id or msg.run_id
        ack_reset(self, self.events_pub, msg)

    def on_vehicles(self, msg):
        now = self.now()
        for v in msg.vehicles:
            obs_t = Time.from_msg(v.header.stamp).nanoseconds / 1e9
            self.estimator.add(v.vehicle_id, v.kind, v.pose.x, v.pose.y, v.speed_mps,
                               v.edge_id, v.valid, obs_t, now)

    def peek_label(self, e):
        """Label without advancing the slow-speed counter (cheap change detection)."""
        if not e.valid:
            return 'UNKNOWN'
        if e.blocked:
            return 'BLOCKED'
        if e.density_ratio >= self.get_parameter('congestion_density_threshold').value:
            return 'CONGESTED'
        return self.labels.get(e.edge_id, 'NORMAL') if self.labels.get(e.edge_id) in (
            'CONGESTED', 'NORMAL') else 'NORMAL'

    def label(self, e):
        if not e.valid:
            return 'UNKNOWN'
        if e.blocked:
            return 'BLOCKED'
        slow = e.vehicle_count > 0 and e.mean_speed_mps < \
            self.get_parameter('slow_speed_ratio').value * FREE_FLOW_MPS
        self.slow_count[e.edge_id] = self.slow_count.get(e.edge_id, 0) + 1 if slow else 0
        if e.density_ratio >= self.get_parameter('congestion_density_threshold').value or \
                self.slow_count[e.edge_id] >= SLOW_PUBLICATIONS:
            return 'CONGESTED'
        return 'NORMAL'

    def tick(self):
        """Evaluate at 10 Hz; publish at 2 Hz and immediately when a label changes."""
        self.ticks += 1
        estimates = self.estimator.estimate(self.now())
        changed = any(self.peek_label(e) != self.labels.get(e.edge_id) for e in estimates)
        if not changed and self.ticks % 5:
            return
        msg = EdgeStateArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for e in estimates:
            s = EdgeState()
            s.header = msg.header
            s.edge_id = e.edge_id
            s.mean_speed_mps = float(e.mean_speed_mps)
            s.density_ratio = float(e.density_ratio)
            s.blocked = e.blocked
            s.valid = e.valid
            msg.edges.append(s)
            label = self.label(e)
            if self.labels.get(e.edge_id) != label:
                self.labels[e.edge_id] = label
                ev = HubEvent()
                ev.header = msg.header
                ev.run_id = self.run_id
                ev.entity_id = e.edge_id
                ev.event = 'EDGE_' + label
                ev.reason = 'density=%.2f speed=%.2f n=%d' % (
                    e.density_ratio, e.mean_speed_mps, e.vehicle_count)
                self.events_pub.publish(ev)
        self.edges_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
