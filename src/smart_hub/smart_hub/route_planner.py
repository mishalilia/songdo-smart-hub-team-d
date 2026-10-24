# Author: Vladislav Li (12250245)
"""
route_planner (scenario S6).

Serves /route/compute using route_engine with live edge states from /traffic/edges:
blocked and invalid edges are removed, stale edge states fail explicitly, congestion
raises the cost. Publishes ROUTE_COMPUTED with the compute latency. See assignment 13, 18.3.
"""
import math
import time

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from smart_hub.hub_map import NODES
from smart_hub.params import check_parameters
from smart_hub.qos import R10, R100
from smart_hub.route_engine import plan
from smart_hub_interfaces.msg import EdgeStateArray, HubEvent
from smart_hub_interfaces.srv import ComputeRoute


class RoutePlanner(Node):

    def __init__(self):
        super().__init__('route_planner')
        self.declare_parameter('replan_improvement_ratio', 0.15)
        self.declare_parameter('use_edge_states', True)
        self.declare_parameter('edge_states_timeout_sec', 1.5)
        check_parameters(self)

        self.edges = {}
        self.edges_receipt = -1e9
        self.events_pub = self.create_publisher(HubEvent, '/hub/events', R100)
        self.create_subscription(EdgeStateArray, '/traffic/edges', self.on_edges, R10)
        self.create_service(ComputeRoute, '/route/compute', self.on_compute)

    def on_edges(self, msg):
        self.edges = {e.edge_id: e for e in msg.edges}
        self.edges_receipt = time.monotonic()

    def on_compute(self, request, response):
        start = time.monotonic()
        self._compute(request, response)
        latency_ms = (time.monotonic() - start) * 1000.0
        ev = HubEvent()
        ev.header.stamp = self.get_clock().now().to_msg()
        ev.entity_id = request.agent_type
        ev.event = 'ROUTE_COMPUTED' if response.success else 'ROUTE_FAILED'
        ev.reason = '%s->%s %s latency_ms=%.2f' % (
            request.start_node, request.goal_node, response.reason, latency_ms)
        self.events_pub.publish(ev)
        return response

    def _compute(self, request, response):
        blocked = set(request.excluded_edges)
        density, speed = {}, {}
        if self.get_parameter('use_edge_states').value:
            if time.monotonic() - self.edges_receipt > \
                    self.get_parameter('edge_states_timeout_sec').value:
                response.success = False
                response.reason = 'EDGE_STATES_STALE'
                return response
            for eid, e in self.edges.items():
                if e.blocked or not e.valid:
                    blocked.add(eid)       # removed, not penalised
                    continue
                density[eid] = e.density_ratio
                if e.density_ratio > 0.0:
                    # Observed traffic: its speed (bounded below by the engine at 0.05).
                    speed[eid] = max(e.mean_speed_mps, 0.05)
        try:
            nodes, cost = plan(request.start_node, request.goal_node, request.agent_type,
                               blocked, density, speed)
        except ValueError as exc:
            response.success = False
            response.reason = f'INVALID_REQUEST: {exc}'
            return response
        if not nodes:
            response.success = False
            response.reason = 'NO_ROUTE'
            return response
        response.success = True
        response.node_ids = nodes
        response.waypoints = [Point(x=NODES[n][0], y=NODES[n][1], z=0.0) for n in nodes]
        response.estimated_cost_sec = float(cost) if math.isfinite(cost) else -1.0
        response.reason = 'OK'
        return response


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
