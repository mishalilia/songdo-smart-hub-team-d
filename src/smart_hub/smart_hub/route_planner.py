"""
route_planner — owner M6 (scenario S6).

Serves /route/compute using route_engine with live edge states from /traffic/edges.
See assignment 13, 18.3.
"""
import math

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from smart_hub.hub_map import NODES
from smart_hub.qos import R10
from smart_hub.route_engine import plan
from smart_hub_interfaces.msg import EdgeStateArray
from smart_hub_interfaces.srv import ComputeRoute


class RoutePlanner(Node):

    def __init__(self):
        super().__init__('route_planner')
        self.declare_parameter('replan_improvement_ratio', 0.15)
        self.declare_parameter('use_edge_states', False)  # TODO(M6): enable when S5 is ready

        self.edges = {}
        self.create_subscription(EdgeStateArray, '/traffic/edges', self.on_edges, R10)
        self.create_service(ComputeRoute, '/route/compute', self.on_compute)

    def on_edges(self, msg):
        self.edges = {e.edge_id: e for e in msg.edges}

    def on_compute(self, request, response):
        blocked = set(request.excluded_edges)
        density, speed = {}, {}
        if self.get_parameter('use_edge_states').value:
            for eid, e in self.edges.items():
                # TODO(M6): stale-critical edges -> blocked; message age check
                if e.blocked or not e.valid:
                    blocked.add(eid)
                else:
                    density[eid] = e.density_ratio
                    speed[eid] = e.mean_speed_mps
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
