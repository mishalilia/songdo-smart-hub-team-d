"""
traffic_monitor — owner M6 (scenario S5).

Per-edge speed, density rho = min(1, n/K), congestion and blockage from /hub/vehicles,
5 s sliding window. Publishes /traffic/edges at 2 Hz. Stale edge => valid=false,
never "uncongested". See assignment 12.
"""
import rclpy
from rclpy.node import Node
from smart_hub.hub_map import edge_id, PARKING_EDGES, VEHICLE_EDGES
from smart_hub.qos import R10
from smart_hub_interfaces.msg import EdgeState, EdgeStateArray, VehicleStateArray


class TrafficMonitor(Node):

    def __init__(self):
        super().__init__('traffic_monitor')
        self.declare_parameter('congestion_density_threshold', 0.70)
        self.declare_parameter('slow_speed_ratio', 0.40)
        self.declare_parameter('window_sec', 5.0)

        self.edge_ids = sorted({edge_id(a, b) for a, b in VEHICLE_EDGES + PARKING_EDGES})
        self.edges_pub = self.create_publisher(EdgeStateArray, '/traffic/edges', R10)
        self.create_subscription(VehicleStateArray, '/hub/vehicles', self.on_vehicles, R10)
        self.create_timer(0.5, self.tick)

    def on_vehicles(self, msg):
        pass  # TODO(M6): validate pose vs. edge corridor, push into sliding window

    def tick(self):
        msg = EdgeStateArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for eid in self.edge_ids:
            e = EdgeState()
            e.header = msg.header
            e.edge_id = eid
            e.valid = False  # UNKNOWN until observations arrive
            msg.edges.append(e)
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
