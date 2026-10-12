"""
dashboard_marker_publisher — owner M8. Read-only: no control authority.

Publishes /hub/markers (5 Hz) for RViz: map nodes/edges, crossing, queue, parking states,
routes, agents, prediction text. See assignment 3.2.
"""
import rclpy
from rclpy.node import Node
from smart_hub.hub_map import NODES
from smart_hub.qos import R10
from visualization_msgs.msg import Marker, MarkerArray


class DashboardMarkerPublisher(Node):

    def __init__(self):
        super().__init__('dashboard_marker_publisher')
        self.markers_pub = self.create_publisher(MarkerArray, '/hub/markers', R10)
        self.create_timer(0.2, self.tick)

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        out = MarkerArray()
        for i, (name, (x, y)) in enumerate(sorted(NODES.items())):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns, m.id = 'nodes', i
            m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.3
            m.pose.orientation.w = 1.0
            m.scale.z = 0.35
            m.color.r = m.color.g = m.color.b = m.color.a = 1.0
            m.text = name
            out.markers.append(m)
        # TODO(M8): edges, crossing/queue regions, parking colours, agents, routes, demand text
        self.markers_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
