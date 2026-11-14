# Author: Vladislav Li (12250245)
"""
route_planner (scenario S6).

Serves /route/compute using route_engine with live edge states from /traffic/edges:
blocked, invalid and unknown edges are removed, congestion raises the cost, and the
expected crossing wait comes from /crossing/state. The edge snapshot must be fresh by both
local receipt time and its observation stamp (edge_states_timeout_sec), otherwise the
request fails with EDGE_STATES_STALE. Every reply is published as ROUTE_COMPUTED /
ROUTE_FAILED with the path, cost, snapshot age and compute latency. See assignment 13, 18.3.
"""
import math
import time

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from smart_hub.freshness import Retained, stamp_problem, stamp_sec
from smart_hub.hub_map import NODES
from smart_hub.params import check_parameters
from smart_hub.qos import R10, R100, R1T
from smart_hub.route_engine import cost_inputs, plan
from smart_hub_interfaces.msg import CrossingState, EdgeStateArray, HubEvent
from smart_hub_interfaces.srv import ComputeRoute

CROSSING_STALE_SEC = 1.0


def snapshot_problem(receipt_age, stamp, now, max_age_sec):
    """Return '' for a usable edge snapshot, else why it is stale (receipt or stamp)."""
    if receipt_age > max_age_sec:
        return 'STALE_RECEIPT'
    return stamp_problem(stamp, now, max_age_sec)


def expected_crossing_wait(crossing, agent_type, wait_sec):
    """
    Return the expected wait before C for one passage (the w_e term of 13.2).

    `crossing` is a fresh, valid CrossingState or None. Zero only when C is known to be
    free of pedestrians and other owners, or the requesting agent already owns it.
    """
    if crossing is None:
        return wait_sec
    if crossing.pedestrian_present:
        return wait_sec
    if crossing.owner_id and not crossing.owner_id.startswith(agent_type):
        return wait_sec
    if not crossing.owner_id and crossing.state != 'FREE':
        return wait_sec
    return 0.0


class RoutePlanner(Node):

    def __init__(self):
        super().__init__('route_planner')
        self.declare_parameter('replan_improvement_ratio', 0.15)
        self.declare_parameter('use_edge_states', True)
        self.declare_parameter('edge_states_timeout_sec', 1.5)
        self.declare_parameter('crossing_wait_sec', 4.0)
        check_parameters(self)

        self.edges = {}
        self.edges_stamp = 0.0
        self.edges_receipt = -1e9
        self.crossing = Retained(CROSSING_STALE_SEC)
        self.events_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.create_subscription(EdgeStateArray, '/traffic/edges', self.on_edges, R10)
        self.create_subscription(CrossingState, '/crossing/state', self.on_crossing, R1T)
        self.create_service(ComputeRoute, '/route/compute', self.on_compute)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_edges(self, msg):
        self.edges = {e.edge_id: e for e in msg.edges}
        self.edges_stamp = stamp_sec(msg.header.stamp)
        self.edges_receipt = time.monotonic()

    def on_crossing(self, msg):
        self.crossing.put(msg, time.monotonic())

    def on_compute(self, request, response):
        start = time.monotonic()
        detail = self._compute(request, response)
        latency_ms = (time.monotonic() - start) * 1000.0
        ev = HubEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.entity_id = request.agent_type
        ev.event = 'ROUTE_COMPUTED' if response.success else 'ROUTE_FAILED'
        ev.reason = '%s->%s %s %s latency_ms=%.2f' % (
            request.start_node, request.goal_node, response.reason, detail, latency_ms)
        self.events_pub.publish(ev)
        return response

    def _compute(self, request, response):
        """Fill the response; return the evidence text for the route event."""
        excluded = sorted(set(request.excluded_edges))
        detail = 'excluded=%s' % (','.join(excluded) or '-')
        states = None
        wait_sec = 0.0
        if self.get_parameter('use_edge_states').value:
            now = self.now()
            age = now - self.edges_stamp
            problem = snapshot_problem(
                time.monotonic() - self.edges_receipt, self.edges_stamp, now,
                self.get_parameter('edge_states_timeout_sec').value)
            if problem:
                response.success = False
                response.reason = 'EDGE_STATES_STALE'
                return '%s snapshot=%s' % (detail, problem)
            states = self.edges
            wait_sec = expected_crossing_wait(
                self.crossing.get(now, time.monotonic()), request.agent_type,
                self.get_parameter('crossing_wait_sec').value)
            detail += ' edges_age_ms=%.0f crossing_wait=%.1fs' % (age * 1000.0, wait_sec)
        try:
            if states is None:
                inputs = (set(excluded), {}, {}, {})
            else:
                inputs = cost_inputs(states, request.agent_type, wait_sec, excluded)
            nodes, cost = plan(request.start_node, request.goal_node, request.agent_type,
                               *inputs)
        except ValueError as exc:
            response.success = False
            response.reason = f'INVALID_REQUEST: {exc}'
            return detail
        if not nodes:
            response.success = False
            response.reason = 'NO_ROUTE'
            return '%s removed=%s' % (detail, ','.join(sorted(inputs[0])) or '-')
        response.success = True
        response.node_ids = nodes
        response.waypoints = [Point(x=NODES[n][0], y=NODES[n][1], z=0.0) for n in nodes]
        response.estimated_cost_sec = float(cost) if math.isfinite(cost) else -1.0
        response.reason = 'OK'
        return 'path=%s cost=%.3fs %s' % ('-'.join(nodes), cost, detail)


def main(args=None):
    rclpy.init(args=args)
    node = RoutePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
